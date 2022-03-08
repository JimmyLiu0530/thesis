/*
    Note that
    (1) if the index is the real index of something, like UE, AP,..., then it will be added with suffix "_idx".
*/



#include <map>
#include <tuple>
#include <limits>
#include <algorithm>
#include <unordered_set>

#include "my_UE_node.h"
#include "proposed_method.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void proposedDynamicLB (int &state,
                           NodeContainer &RF_AP_node,
                           NodeContainer &VLC_AP_nodes,
                           NodeContainer &UE_nodes,
                           std::vector<std::vector<double>> &VLC_LOS_matrix,
                           std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                           std::vector<double> &RF_data_rate_vector,
                           std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                           std::vector<std::vector<int>> &AP_association_matrix,
                           std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                           std::vector<double> demand_discount_per_AP,
                           std::vector<std::tuple<int, int>> &first_empty_RU_position,
                           std::vector<MyUeNode> &my_UE_list)
{
    std::vector<int> rejected_UE;

    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);

    if (state == 0)
        proposedMethodForState0(RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, resource_unit_matrix_per_VLC_AP,
                                 rejected_UE, demand_discount_per_AP, first_empty_RU_position, my_UE_list);
    else
        proposedMethodForStateN(RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, resource_unit_matrix_per_VLC_AP,
                                 rejected_UE, demand_discount_per_AP, first_empty_RU_position, my_UE_list);


#if DEBUG_MODE
    std::cout << "the number of UEs being rejected: " << rejected_UE.size() << std::endl;

    for (int i = 0; i < VLC_AP_num; i++) {
        printResourceUnitMatrix(resource_unit_matrix_per_VLC_AP, i);
    }
#endif // DEBUG_MODE

}


// rewrite
void proposedMethodForState0(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                             std::vector<int> &rejected_UE,
                             std::vector<double> &demand_discount_per_AP,
                             std::vector<std::tuple<int, int>> &first_empty_RU_position,
                             std::vector<MyUeNode> &my_UE_list)
{
    // pre-step: clear AP association matrix and RU matrix
    for (int i = 0; i < AP_association_matrix.size(); i++)
        for (int j = 0; j < AP_association_matrix[i].size(); j++)
            AP_association_matrix[i][j] = 0;

    for (int i = 0; i < RU_matrix_per_VLC_AP.size(); i++) {
        first_empty_RU_position[i] = std::make_tuple(effective_subcarrier_num, time_slot_num-1);

        for (int j = 0; j < RU_matrix_per_VLC_AP[i].size(); j++)
            for (int k = 0; k < RU_matrix_per_VLC_AP[i][j].size(); k++)
                RU_matrix_per_VLC_AP[i][j][k] = 0;
    }


    // step1: find the AP with the best SINR for each UE
    std::vector<int> served_by_RF;
    std::vector<int> best_SINR_AP (UE_num, -1);
    std::vector<std::unordered_set<int>> unallocated_UE_under_best_VLC_AP(VLC_AP_num, std::unordered_set<int> ());
    findBestSinrAP(best_SINR_AP, unallocated_UE_under_best_VLC_AP, served_by_RF, VLC_SINR_matrix);


    // step2: record user's demand in another space and then sort users in the descending order based on their demands
    std::vector<double> UE_demand (UE_num, 0.0);
    for (int i = 0; i < UE_num; i++)
        UE_demand.push_back(my_UE_list[i].getRequiredDataRate());

    // note that be careful of the order of my_UE_list from now on
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getRequiredDataRate() > b.getRequiredDataRate();});


    // step3: APA + RA
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<std::vector<int>> serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();

        if (best_SINR_AP[UE_idx] != 0) {
            /* APA */
            // <chosen AP, offered data rate> is returned
            std::tuple<int, double> result = accessPointAssociation(VLC_data_rate_matrix, RF_data_rate_vector, served_by_RF, RU_matrix_per_VLC_AP, unallocated_UE_under_best_VLC_AP,
                                                                    UE_demand, demand_discount_per_AP, first_empty_RU_position, my_UE_list[i]);
            int chosen_AP = get<0>(result);

            if (chosen_AP < 0) // no serving AP
                rejected_UE.push_back(UE_idx);
            else {
                // update APA result
                AP_association_matrix[chosen_AP][UE_idx] = 1;
                my_UE_list[i].setCurrAssociatedAP(chosen_AP);
                serving_UE[chosen_AP].push_back(UE_idx);

                /* VLC RA */
                if (chosen_AP > 0)
                    throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[chosen_AP-RF_AP_num], RU_matrix_per_VLC_AP[chosen_AP-RF_AP_num], get<1>(result),
                                                            first_empty_RU_position[chosen_AP-RF_AP_num], my_UE_list[i]);
            }
        }
        else {
            int chosen_AP = best_SINR_AP[UE_idx];  // should be 0

            AP_association_matrix[chosen_AP][UE_idx] = 1;
            my_UE_list[i].setCurrAssociatedAP(chosen_AP);
            serving_UE[chosen_AP].push_back(UE_idx);
        }
    }

    for (int i = 0; i < served_by_RF.size(); i++)
        throughput[served_by_RF[i]] = RF_data_rate_vector[served_by_RF.size()];


    //step4: RRA


}

void findBestSinrAP(std::vector<int> &best_SINR_AP,
                    std::vector<std::unordered_set<int>> &unallocated_UE_under_best_VLC_AP,
                    std::vector<int> &served_by_RF,
                    std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix)
{
    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        int best_VLC_AP = -1;
        double best_SINR = 1.0;

        for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
            // if the highest freq subcarrier can't provide UE with any data rate, then no other subcarrier can!
            // that is, this AP cannot serve this UE
            if (VLC_SINR_matrix[VLC_AP_idx][UE_idx][1] < 1.0)
                continue;

            double SINR_sum = 0.0;
            for (int subcarrier_idx = 1; subcarrier_idx <= effective_subcarrier_num; subcarrier_idx++)
                SINR_sum += VLC_SINR_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];

            if (SINR_sum > best_SINR) {
                best_VLC_AP = VLC_AP_idx;
                best_SINR = SINR_sum;
            }
        }

        // no VLC AP can provide data rate for this UE, so choose RF AP as its best AP
        if (best_VLC_AP == -1) {
            best_SINR_AP[UE_idx] = 0;
            served_by_RF.push_back(UE_idx);
        }
        else {
            best_SINR_AP[UE_idx] = best_VLC_AP + RF_AP_num;
            unallocated_UE_under_best_VLC_AP[best_VLC_AP].insert(UE_idx);
        }
    }
}

std::tuple<int, double> accessPointAssociation(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                                std::vector<double> &RF_data_rate_vector,
                                                std::vector<int> &served_by_RF;
                                                std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                                                std::vector<std::unordered_set<int>> &unallocated_UE_under_best_VLC_AP,
                                                std::vector<double>& UE_demand,
                                                std::vector<double> demand_discount_per_AP,
                                                std::vector<std::tuple<int, int>> &first_empty_RU_position,
                                                MyUeNode &UE_node)
{
    int UE_idx = UE_node.getID();
    std::vector<std::vector<std::vector<int>>> RU_matrix_per_VLC_AP_copy = RU_matrix_per_VLC_AP;
    std::vector<std::tuple<int, double>> AP_tuple; // record <number of residual RUs, offered data rate>

    // step1: for each VLC AP allocate resource to those unallocated UEs first, and then the current UE
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] == 0.0) {
            AP_tuple.push_back(std::tuple<int, double> (std::make_tuple(0, 0.0)));
            continue;
        }

        int skip_flag = 0;
        std::tuple<int, int> first_empty_RU_position_copy = first_empty_RU_position[i];

        // step1-1: allocate resource to those unallocated UEs
        for (auto it = unallocated_UE_under_best_VLC_AP.begin(); it != unallocated_UE_under_best_VLC_AP.end(); it++) {
            int update_flag = 1;
            int subcarrier_idx = get<0>(first_empty_RU_position_copy);
            int time_slot_idx = get<1>(first_empty_RU_position_copy);
            double discounted_demand = UE_demand[*it] * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];

            // find the first subcarrier that provides data rate for UE *it
            findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][*it], subcarrier_index, time_slot_idx, update_flag);

            while (discounted_demand > 0 && subcarrier_idx >= 1) {
                if (RU_matrix_per_VLC_AP_copy[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                    discounted_demand -= VLC_data_rate_matrix[VLC_AP_idx][*it][subcarrier_idx];
                    RU_matrix_per_VLC_AP_copy[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                }
                goToNextMove(subcarrier_idx, time_slot_idx);
            }

            // since UEs can get at most their demand multiplied by demand discount, return one RU back
            if (discounted_demand < 0) {
                backToLastRU(subcarrier_idx, time_slot_idx);
                RU_matrix_per_VLC_AP_copy[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
            }

            if (update_flag)
                first_empty_RU_position_copy = std::make_tuple(subcarrier_idx, time_slot_idx);

            // AP has no residual resource
            if (get<0>(first_empty_RU_position_copy) == 0) {
                AP_tuple.push_back(std::tuple<int, double> (0, 0.0));
                skip_flag = 1;
                break;
            }
        }

        // step1-3: allocate resource to the current UE
        if (!skip_flag) {
            int update_flag = 1;
            int subcarrier_idx = get<0>(first_empty_RU_position_copy);
            int time_slot_idx = get<1>(first_empty_RU_position_copy);
            double offered_data_rate = 0.0;
            double discounted_demand = UE_node.getRequiredDataRate() * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];


            findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_index, time_slot_idx, update_flag);

            while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
                if (RU_matrix_per_VLC_AP_copy[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                    offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                    RU_matrix_per_VLC_AP_copy[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                }
                goToNextMove(subcarrier_idx, time_slot_idx);
            }

            // since UEs can get at most their demand multiplied by demand discount, return one RU backs
            if (offered_data_rate > discounted_demand) {
                backToLastRU(subcarrier_idx, time_slot_idx);
                offered_data_rate -= VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                RU_matrix_per_VLC_AP_copy[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
            }

            int empty_RU_num = 0;
            for (int i = 1; i <= effective_subcarrier_num; i++) {
                for (int j = 0; j < time_slot_num; j++) {
                    if (RU_matrix_per_VLC_AP_copy[i][j] == 0)
                        empty_RU_num++;
                }
            }

            AP_tuple.push_back(std::make_tuple(empty_RU_num, offered_data_rate));

            if (update_flag)
                first_empty_RU_position_copy = std::make_tuple(subcarrier_idx, time_slot_idx);
        }
    }

    // step2: after all VLC APs finished allocation, choose the one with max number of residual RU and offered data rate
    int result_AP = -1;
    double max_value = -1;

    for (int VLC_AP_idx = 0; VLC_AP_idx < AP_tuple.size(); VLC_AP_idx++) {
        if (get<1>AP_tuple[i] == 0.0)
            continue;

        double value = (1-weight) * get<0>AP_tuple[i] + weight * get<1>AP_tuple[i];
        if (value > max_value) {
            result_AP = VLC_AP_idx;
            max_value = value;
        }
    }

    if (max_value > 0.0)
        return std::tuple<int, double> (std::make_tuple(result_AP, get<1>AP_tuple[result_AP]));


    // step3: if no VLC AP can serve this UE, then try RF AP
    int i = 0;
    for (; i < served_by_RF.size() && satisfied_flag; i++) {
        if (UE_demand[served_by_RF[i]]*demand_discount_per_AP[0] < RF_data_rate_vector[served_by_RF.size()])
            break;
    }

    if (i == served_by_RF.size() && UE_demand[UE_idx]*demand_discount_per_AP[0] >= RF_data_rate_vector[served_by_RF.size()+1]) {
        served_by_RF.push_back(UE_idx);

        return std::tuple<int, double> (std::make_tuple(0, RF_data_rate_vector[served_by_RF.size()+1]);
    }

    return std::tuple<int, double> (std::make_tuple(-1, 0.0);
}

double resourceAllocation(std::vector<std::vector<double>> &VLC_data_rate_matrix,
                            std::vector<std::vector<int>> &RU_matrix,
                            std::tuple<int, int> &first_empty_RU_position,
                            double offered_data_rate,
                            MyUeNode &UE_node)
{
    int update_flag = 1;
    int UE_idx = UE_node.getID();
    int subcarrier_idx = get<0>(first_empty_RU_position);
    int time_slot_idx = get<1>(first_empty_RU_position);
    std::tuple<int, int> start;

    findFirstEffectiveSubcarrier(VLC_data_rate_matrix[UE_idx], subcarrier_idx, time_slot_idx, update_flag);

    start = std::make_tuple(subcarrier_idx, time_slot_idx);

    while (offered_data_rate > 0 && subcarrier_idx >= 1) {
        if (RU_matrix[subcarrier_idx][time_slot_idx] == 0) {
            offered_data_rate -= VLC_data_rate_matrix[UE_idx][subcarrier_idx];
            RU_matrix[subcarrier_idx][time_slot_idx] = 1;

            goToNextRU(subcarrier_idx, time_slot_idx);
        }
        // reach an RU that has been used, so record the range of RU this UE uses so far and push back to its RU_used vector
        else {
            std::tuple<int, int> tail;
            tail = (time_slot_idx == time_slot_num-1) ? std::make_tuple(subcarrier_idx+1, 0) : std::make_tuple(subcarrier_idx, time_slot_idx+1);
            UE_node.useResourceUnit(std::make_tuple(start, tail));

            // find the first empty RU
            while (subcarrier_idx >= 1 && RU_matrix[subcarrier_idx][time_slot_idx] == 1)
                goToNextRU(subcarrier_idx, time_slot_idx);

            start = std::make_tuple(subcarrier_idx, time_slot_idx);
        }
    }

    double throughput = offered_data_rate;

    // since UEs can get at most their demand multiplied by demand discount, return one RU back
    if (offered_data_rate < 0) {
        backToLastRU(subcarrier_idx, time_slot_idx);
        RU_matrix[subcarrier_idx][time_slot_idx] = 0;
        throughput -= VLC_data_rate_matrix[UE_idx][subcarrier_idx];
    }

    if (update_flag)
        first_empty_RU_position = std::make_tuple(subcarrier_idx, time_slot_idx);

    backToLastRU(subcarrier_idx, time_slot_idx); // we have to go back one RU, since the pointer stops at one RU ahead of the last RU this UE uses
    UE_node.useResourceUnit(std::make_tuple(start, std::make_tuple(subcarrier_idx, time_slot_idx)));

    return throughput;
}

// find the first subcarrier that provides data rate for the specific UE
// if this UE does not use the first empty RU, then it's unnecessary for first_empty_RU_position to update
void findFirstEffectiveSubcarrier(std::vector<double> &VLC_data_rate_matrix, int &subcarrier_index, int &time_slot_idx, int &update_flag)
{
    while (subcarrier_index >= 1 && VLC_data_rate_matrix[subcarrier_idx] == 0.0) {
        update_flag = 0;
        subcarrier_idx--;
        time_slot_idx = time_slot_num - 1;
    }
}

void backToLastRU(int &subcarrier_idx, int &time_slot_idx)
{
    if (time_slot_idx == time_slot_num - 1) {
            subcarrier_idx++;
            time_slot_idx = 0;
    }
    else
        time_slot_idx++;
}

void goToNextRU(int &subcarrier_idx, int &time_slot_idx)
{
    time_slot_idx--;

    if (time_slot_idx == -1) {
        time_slot_idx = time_slot_num - 1;
        subcarrier_idx--;
    }
}


/* ----------------------------------------------------------------------------------------------------------------------- */


// assume we periodically call this algorithm to update
void proposedMethodForState0_old(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<int> &rejected_UE,
                             std::vector<MyUeNode> &my_UE_list)
{
    // pre-step: clear AP_association_matrix and resource_unit_matrix_per_VLC_AP
    for (int i = 0; i < AP_association_matrix.size(); i++)
        for (int j = 0; j < AP_association_matrix[i].size(); j++)
            AP_association_matrix[i][j] = 0;

     for (int i = 0; i < resource_unit_matrix_per_VLC_AP.size(); i++)
        for (int j = 0; j < resource_unit_matrix_per_VLC_AP[i].size(); j++)
                for (int k = 0; k < resource_unit_matrix_per_VLC_AP[i][j].size(); k++)
                    resource_unit_matrix_per_VLC_AP[i][j][k] = 0;


    // step1: find the VLC AP with the best SINR for each UE. If the max SINR is smaller than 1, then choose the RF AP.
    std::vector<int> best_SINR_AP (UE_num, -1);

    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        int best_AP = 0;
        double max_SINR = 0.0;

        for (int subcarrier_idx = 1; subcarrier_idx <= effective_subcarrier_num; subcarrier_idx++)
            max_SINR += VLC_SINR_matrix[best_AP][UE_idx][subcarrier_idx];

        for (int AP_idx = 1; AP_idx < VLC_AP_num; AP_idx++) {
            double total_SINR = 0.0;
            for (int subcarrier_idx = 1; subcarrier_idx <= effective_subcarrier_num; subcarrier_idx++)
                total_SINR += VLC_SINR_matrix[AP_idx][UE_idx][subcarrier_idx];

            if (total_SINR > max_SINR) {
                best_AP = AP_idx;
                max_SINR = total_SINR;
            }
        }

        // here means no VLC AP can provide data rate for this UE, so choose RF AP as its best AP
        if (max_SINR < 1.0)
            best_SINR_AP[UE_idx] = 0;
        else
            best_SINR_AP[UE_idx] = best_AP + RF_AP_num; // VLC APs are indexed from 1
    }


    // step2: record the set of UEs who has the the best SINR with VLC AP i, UEs connect to the RF AP and UE's demand
    std::vector<int> served_by_RF;
    std::vector<double> UE_demand;
    std::vector<std::unordered_set<int>> unallocated_UE_under_VLC_AP (VLC_AP_num, std::unordered_set<int> ());

    for (int UE_idx = 0; UE_idx < my_UE_list.size(); UE_idx++) {
        if (best_SINR_AP[UE_idx] == 0)
            served_by_RF.push_back(UE_idx);
        else {
            int VLC_AP_index = best_SINR_AP[UE_idx] - RF_AP_num;
            unallocated_UE_under_VLC_AP[VLC_AP_index].insert(UE_idx);
        }

        UE_demand.push_back(my_UE_list[UE_idx].getRequiredDataRate());
    }

    // sort UEs in the descending order based on their demands
    // note that be careful of the order of my_UE_list from on on
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getRequiredDataRate() > b.getRequiredDataRate();});



    // step3: APA+RA
    std::vector<double> throughput (UE_num, 0.0); // record the thoughput each UE would get in this step
    std::vector<std::vector<int>> serving_UE(RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int i = 0; i < my_UE_list.size(); i++) {
        /* APA */
        int chosen_AP = -1;
        int UE_idx = my_UE_list[i].getID();

        // if best AP is not RF AP:
        //  find the AP with max residual resource after fulfilling this user's demand
        //  and choose it as the user's serving AP.
        if (best_SINR_AP[UE_idx] != 0) {
            std::vector<int> VLC_AP_order = sortApBasedOnResidualResource(VLC_data_rate_matrix, resource_unit_matrix_per_VLC_AP, my_UE_list[i]);
            auto it = VLC_AP_order.begin();

            while (it != VLC_AP_order.end()) {
                if ((*it) + RF_AP_num == best_SINR_AP[UE_idx]) {
                    chosen_AP = best_SINR_AP[UE_idx];
                    break;
                }
                else { // consider whether the UEs under this AP will become unsatisfactory if allocating some resource to this UE
                    if (checkSatisfactionUnderAP(VLC_data_rate_matrix, resource_unit_matrix_per_VLC_AP, unallocated_UE_under_VLC_AP[*it], UE_demand, *it, UE_idx)) {
                        chosen_AP = (*it) + RF_AP_num;
                        break;
                    }
                }
                it++;
            }

            // try RF AP
            if (chosen_AP == -1) {
                int satisfied_flag = 1; // whether all UEs under the RF AP get satisfied
                for (int i = 0; i < served_by_RF.size() && satisfied_flag; i++) {
                    if (UE_demand[served_by_RF[i]] < RF_data_rate_vector[served_by_RF.size()])
                        satisfied_flag = 0;
                }

                if (satisfied_flag && UE_demand[UE_idx] >= RF_data_rate_vector[served_by_RF.size()+1]) {
                    chosen_AP = 0;
                    served_by_RF.push_back(UE_idx);
                }
            }

            // update APA result to my_UE_list and AP_association_matrix
            unallocated_UE_under_VLC_AP[best_SINR_AP[UE_idx] - RF_AP_num].erase(UE_idx);
            my_UE_list[UE_idx].setCurrAssociatedAP(chosen_AP);

            if (chosen_AP != -1) {
                AP_association_matrix[chosen_AP][UE_idx] = 1;
                serving_UE[chosen_AP].push_back(UE_idx);

                if (chosen_AP == 0)
                    continue;
            }
            else { // this UE has no association with AP in this state
                rejected_UE.push_back(UE_idx);
                continue;
            }


            /* (VLC AP) RA */

            int subcarrier_index = effective_subcarrier_num;
            int time_slot_index = time_slot_num - 1;
            double offered_data_rate = 0.0;
            std::vector<std::vector<int>> resource_unit_matrix = resource_unit_matrix_per_VLC_AP[chosen_AP - RF_AP_num];

            // find the first subcarrier that provides data rate for UE
            while (subcarrier_index >= 1 && VLC_data_rate_matrix[chosen_AP - RF_AP_num][UE_idx][subcarrier_idx] == 0.0)
                subcarrier_idx--;

            // find the beginning of the unused RU
            while (subcarrier_index >= 1 && resource_unit_matrix[subcarrier_index][time_slot_index] == 1) {
                time_slot_index--;

                if (time_slot_index == -1) {
                    time_slot_index = time_slot_num - 1;
                    subcarrier_index--;
                }
            }

            while (offered_data_rate < UE_demand[UE_idx] && subcarrier_index >= 1) {
                if (resource_unit_matrix[subcarrier_idx][time_slot_idx] == 0) {
                    offered_data_rate += VLC_data_rate_matrix[chosen_AP - RF_AP_num][UE_idx][subcarrier_idx];
                    resource_unit_matrix[subcarrier_idx][time_slot_idx] = 1;
                }
                time_slot_idx--;

                if (time_slot_idx == -1) {
                    time_slot_idx = time_slot_num - 1;
                    subcarrier_idx--;
                }
            }

            // this AP cannot satisfies the demand of this UE although we give it the rest of all resource, so reject this UE's request
            if (offered_data_rate < UE_demand[UE_idx]) {
                rejected_UE.push_back(UE_idx);
                offered_data_rate = 0.0;
            }
            else {
                resource_unit_matrix_per_VLC_AP[chosen_AP - RF_AP_num] = resource_unit_matrix; // update resource_unit_matrix of this AP
            }

            throughput[UE_idx] = offered_data_rate;
        }
        else { // best AP is RF_AP
            chosen_AP = best_SINR_AP[UE_idx]; // 0

            AP_association_matrix[chosen_AP][UE_idx] = 1;
            my_UE_list[i].setCurrAssociatedAP(chosen_AP);
            serving_UE[chosen_AP].push_back(UE_idx);
        }
    }

    // RA of RF AP is done until the end of step3
    // since the number of UEs served by RF AP may change during step3
    for (int i = 0; i < served_by_RF.size(); i++) {
        throughput[served_by_RF[i]] = RF_data_rate_vector[served_by_RF.size()];
    }


    // step4: residual resource allocation for VLC APs
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        int subcarrier_start = effective_subcarrier_num;
        int time_slot_start = time_slot_num - 1;
        std::vector<std::vector<int>> resource_unit_matrix = resource_unit_matrix_per_VLC_AP[VLC_AP_idx];

        // find the beginning of the unused RU
        while (subcarrier_start >= 1 && resource_unit_matrix[subcarrier_start][time_slot_start] == 1) {
            time_slot_start--;

            if (time_slot_start == -1) {
                time_slot_start = time_slot_num - 1;
                subcarrier_start--;
            }
        }

        // distribute residual resource in the ascending order of the difference between demand and throughput UE has now
        sort(serving_UE[VLC_AP_idx].begin(), serving_UE[VLC_AP_idx].end(),
             [](int a, int b){return (UE_demand[a] - throughput[a]) < (UE_demand[b] - throughput[b]);});

        int i = 0;
        for (; i < serving_UE[VLC_AP_idx].size(); i++) {
            int UE_idx = serving_UE[VLC_AP_idx][i];
            int subcarrier_idx = subcarrier_start;
            int time_slot_idx = time_slot_start;

            // find the first subcarrier that provides data rate for UE *it
            while (subcarrier_index >= 1 && VLC_data_rate_matrix[VLC_AP_index][UE_idx][subcarrier_idx] == 0.0) {
                subcarrier_idx--;
                time_slot_idx = time_slot_num - 1;
            }

            while (throughput[UE_idx] < UE_demand[UE_idx] && subcarrier_idx >= 1) {
                if (resource_unit_matrix[subcarrier_idx][time_slot_idx] == 0) {
                    throughput[UE_idx] += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                    resource_unit_matrix[subcarrier_idx][time_slot_idx] = 1;
                }
                time_slot_idx--;

                if (time_slot_idx == -1) {
                    time_slot_idx = time_slot_num - 1;
                    subcarrier_idx--;
                }
            }

            my_UE_list[UE_idx].addThroughput(throughput[UE_idx]);
            my_UE_list[UE_idx].addSatisfaction(throughput[UE_idx] / UE_demand[UE_idx]);

            // no resource of this VLC AP is left
            if (throughput[UE_idx] < UE_demand[UE_idx]) {
                break;
            }
        }

        for (; i < serving_UE[VLC_AP_idx].size(); i++) {
            int UE_idx = serving_UE[VLC_AP_idx][i];

            my_UE_list[UE_idx].addThroughput(throughput[UE_idx]);
            my_UE_list[UE_idx].addSatisfaction(throughput[UE_idx] / UE_demand[UE_idx]);
        }
    }
}

// return true if not affected
bool checkSatisfactionUnderAP(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                              std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                              std::unordered_set<int> &unallocated_UE_set,
                              std::vector<double> &UE_demand,
                              int VLC_AP_index,
                              int UE_index)
{
    std::vector<std::vector<int>> resource_unit_matrix = RU_matrix_per_VLC_AP[VLC_AP_index];

    // at first, allocate resource to those unallocated UEs
    int subcarrier_start = effective_subcarrier_num;
    int time_slot_start = time_slot_num - 1;

    while (subcarrier_start >= 1 && resource_unit_matrix[subcarrier_start][time_slot_start] == 1) {
        time_slot_start--;

        if (time_slot_start == -1) {
            time_slot_start = time_slot_num - 1;
            subcarrier_start--;
        }
    }

    for (auto it = unallocated_UE_set.begin(); it != unallocated_UE_set.end(); it++) {
        int subcarrier_idx = subcarrier_start;
        int time_slot_idx = time_slot_start;
        double demand = UE_demand[*it];

        // find the first subcarrier that provides data rate for UE *it
        while (subcarrier_index >= 1 && VLC_data_rate_matrix[VLC_AP_index][*it][subcarrier_idx] == 0.0) {
            subcarrier_idx--;
            time_slot_idx = time_slot_num - 1;
        }

        while (demand > 0 && subcarrier_idx >= 1) {
            if (resource_unit_matrix[subcarrier_idx][time_slot_idx] == 0) {
                demand -= VLC_data_rate_matrix[VLC_AP_index][*it][subcarrier_idx];
                resource_unit_matrix[subcarrier_idx][time_slot_idx] = 1;
            }
            time_slot_idx--;

            if (time_slot_idx == -1) {
                time_slot_idx = time_slot_num - 1;
                subcarrier_idx--;
            }
        }

        if (demand > 0)
            return false;
    }

    // then check if this incoming UE can be satisfied
    int subcarrier_idx = subcarrier_start;
    int time_slot_idx = time_slot_start;
    double demand = UE_demand[UE_index];

    // find the first subcarrier that provides data rate for UE *it
    while (subcarrier_index >= 1 && VLC_data_rate_matrix[VLC_AP_index][UE_index][subcarrier_idx] == 0.0) {
        subcarrier_idx--;
        time_slot_idx = time_slot_num - 1;
    }

    while (demand > 0 && subcarrier_idx >= 1) {
        if (resource_unit_matrix[subcarrier_idx][time_slot_start] == 0) {
            demand -= VLC_data_rate_matrix[VLC_AP_index][UE_index][subcarrier_idx];
            resource_unit_matrix[subcarrier_idx][time_slot_start] = 1;
        }
        time_slot_idx--;

        if (time_slot_idx == -1) {
            time_slot_idx = time_slot_num - 1;
            subcarrier_idx--;
        }
    }

    if (demand > 0)
        return false;
    else
        return true;
}

// sort VLC APs in the descending order based on their residual resource after fulfilling UE's demand
std::vector<int> sortApBasedOnResidualResource(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                               std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                                               MyUeNode &UE)
{
    std::map<int,int,std::greater<int>> residual_RU_num; // {the number of residual RUs, VLC AP index}
    std::vector<double> offered_data_rate_per_AP;

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        int UE_idx = UE.getID();
        int subcarrier_idx = effective_subcarrier_num;
        int time_slot_idx = time_slot_num - 1;
        double UE_demand = UE.getRequiredDataRate();
        double offered_data_rate = 0.0;
        std::vector<std::vector<int>> resource_unit_matrix = RU_matrix_per_VLC_AP[VLC_AP_idx];

        // find the first subcarrier that provides data rate for UE
        while (subcarrier_index >= 1 && VLC_data_rate_matrix[VLC_AP_index][UE_idx][subcarrier_idx] == 0.0)
            subcarrier_idx--;

        // find the beginning of the unused RU
        while (subcarrier_idx >= 1 && resource_unit_matrix[subcarrier_idx][time_slot_idx] == 1) {
            time_slot_idx--;

            if (time_slot_idx == -1) {
                time_slot_idx = time_slot_num - 1;
                subcarrier_idx--;
            }
        }

        // try to fulfill user's demand from high-freq subcarrier to low-freq
        while (offered_data_rate < UE_demand && subcarrier_idx >= 1) {
            if (resource_unit_matrix[subcarrier_idx][time_slot_idx] == 0) {
                offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                resource_unit_matrix[subcarrier_idx][time_slot_idx] = 1;
            }
            time_slot_idx--;

            if (time_slot_idx == -1) {
                time_slot_idx = time_slot_num - 1;
                subcarrier_idx--;
            }
        }
        offered_data_rate_per_AP[VLC_AP_idx] = offered_data_rate;

        // this AP fulfills the user's demand, then count how many RUs are left
        if (offered_data_rate >= UE_demand) {
            int counter = 0;
            for (int i = 1; i <= effective_subcarrier_num; i++) {
                for (int j = 0; j < time_slot_num; j++) {
                    if (resource_unit_matrix[i][j] == 0)
                        counter++;
                }
            }
            residual_RU_num[counter] = VLC_AP_idx;
        }
    }


    std::vector<int> result;

    for (auto it = residual_RU_num.begin(); it != residual_RU_num.end(); it++) {
        result.push_back(it->second);
    }

    return result;
}


// consider handover effieicney
void proposedMethodForStateN(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                             std::vector<int> &rejected_UE,
                             std::vector<double> &demand_discount_per_VLC_AP,
                             std::vector<std::tuple<int, int>> &first_empty_RU_position,
                             std::vector<MyUeNode> &my_UE_list)
{

}

