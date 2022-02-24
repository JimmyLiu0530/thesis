#include <algorithm>
#include <limits>
#include <map>
#include <unordered_set>

#include "my_UE_node.h"
#include "proposed_method.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


double proposedDynamicLB (int &state,
                           NodeContainer &RF_AP_node,
                           NodeContainer &VLC_AP_nodes,
                           NodeContainer &UE_nodes,
                           std::vector<std::vector<double>> &VLC_LOS_matrix,
                           std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                           std::vector<double> &RF_data_rate_vector,
                           std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                           std::vector<std::vector<int>> &AP_association_matrix,
                           std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                           std::vector<MyUeNode> &my_UE_list)
{
    std::vector<int> blocked_UE;

    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);

    if (state == 0)
        proposedMethodForState0(RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, resource_unit_matrix_per_VLC_AP, blocked_UE, my_UE_list);
    else
        proposedMethodForStateN(RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, resource_unit_matrix_per_VLC_AP, blocked_UE, my_UE_list);


#if DEBUG_MODE
    for (int i = 0; i < VLC_AP_num; i++) {
        printResourceUnitMatrix(resource_unit_matrix_per_VLC_AP, i);
    }

    printBlockedUE(blocked_UE);
#endif // DEBUG_MODE

}


// assume we periodically call this algorithm to update
void proposedMethodForState0(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<int> &blocked_UE,
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
        else {
            best_SINR_AP[UE_idx] = best_AP + RF_AP_num; // VLC APs are indexed from 1
            total_demand_under_VLC_AP[best_AP] += my_UE_list[UE_idx].getRequiredDataRate();
        }
    }


    // step2: sort UEs in the descending order based on their demands
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getRequiredDataRate() > b.getRequiredDataRate();});

    // record the set of UEs who has the the best SINR with VLC AP i and calculate how many UEs connect to the RF AP
    int RF_UE_num = 0;
    std::vector<std::unordered_set<int>> unallocated_UE_under_VLC_AP (VLC_AP_num, std::unordered_set<int> ());

    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_index = my_UE_list[i].getID();

        if (best_SINR_AP[UE_index] == 0)
            RF_UE_num++;
        else {
            int VLC_AP_index = best_SINR_AP[UE_index] - RF_AP_num;
            unallocated_UE_under_VLC_AP[VLC_AP_index].insert(UE_index);
        }
    }


    // step3: APA+RA
    for (int UE_idx = 0; UE_idx < my_UE_list.size(); UE_idx++) {
        /* APA */
        int chosen_AP = -1;
        int UE_real_index = my_UE_list[UE_idx].getID();
        // best AP is not RF AP:
        //  find the AP with max residual resource after fulfilling the user u's demand
        //  and choose it as the user's serving AP.
        if (best_SINR_AP[UE_real_index] != 0) {
            std::vector<int> VLC_AP_order = sortApBasedOnResidualResource(VLC_data_rate_matrix, resource_unit_matrix_per_VLC_AP, my_UE_list[UE_idx]);
            auto it = VLC_AP_order.begin();

            while (it != VLC_AP_order.end()) {
                if ((*it) + RF_AP_num == best_SINR_AP[mUE_real_index]) {
                    chosen_AP = best_SINR_AP[UE_real_index];
                    break;
                }
                else { // have to consider whether the UEs under this AP will become unsatisfactory if allocating some resource to this UE
                    if (checkSatisfactionUnderAP(VLC_data_rate_matrix, resource_unit_matrix_per_VLC_AP, unallocated_UE_under_VLC_AP[*it], my_UE_list, *it, UE_real_index)) {
                        chosen_AP = (*it) + RF_AP_num;
                        break;
                    }
                }
                it++;
            }

            unallocated_UE_under_VLC_AP[best_SINR_AP[UE_real_index] - RF_AP_num].erase(UE_idx);
            my_UE_list[UE_idx].setCurrAssociatedAP(chosen_AP);

            if (chosen_AP != -1) {
                AP_association_matrix[chosen_AP][UE_real_index] = 1;
            }
            else { // this UE has no association with AP in this state
                blocked_UE.push_back(UE_real_index);
                continue;
            }


            /* RA */
            std::vector<std::vector<int>> resource_unit_matrix = resource_unit_matrix_per_VLC_AP[chosen_AP];
            double demand = my_UE_list[UE_idx].getRequiredDataRate();
            double offered_data_rate = 0.0;
            int subcarrier_index = effective_subcarrier_num;
            int time_slot_index = time_slot_num - 1;

            // find the beginning of the resource_unit_matrix
            while (subcarrier_index > 1 && resource_unit_matrix[subcarrier_index][time_slot_index] == 1) {
                time_slot_index--;

                if (time_slot_index == -1) {
                    time_slot_index = time_slot_num - 1;
                    subcarrier_index--;
                }
            }

            while ((offered_data_rate < demand) && (subcarrier_index > 1)) {
                if (resource_unit_matrix[subcarrier_idx][time_slot_idx] == 0) {
                    offered_data_rate += VLC_data_rate_matrix[chosen_AP-RF_AP_num][UE_real_index][subcarrier_idx];
                    resource_unit_matrix[subcarrier_idx][time_slot_idx] = 1;
                }
                time_slot_idx--;

                if (time_slot_idx == -1) {
                    time_slot_idx = time_slot_num - 1;
                    subcarrier_idx--;
                }
            }

            // this AP cannot satisfies the demand of this UE although we give it the rest of all resource, so
            if (offered_data_rate < demand) {
                blocked_UE.push_back(UE_real_index);
                offered_data_rate = 0.0;
            }
            else {
                resource_unit_matrix_per_VLC_AP[chosen_AP] = resource_unit_matrix; // update resource_unit_matrix of this AP
            }

            my_UE_list[UE_idx].addThroughput(offered_data_rate);
        }
        else { // best AP is RF_AP
            chosen_AP = best_SINR_AP[my_UE_list[UE_idx].getID()];
            AP_association_matrix[chosen_AP][UE_real_index] = 1;
            my_UE_list[UE_idx].setCurrAssociatedAP(chosen_AP);

            /* RA */
            my_UE_list[UE_idx].addThroughput(RF_data_rate_vector[RF_UE_num]);
        }
    }


    // step4: residual resource allocation

}

// return true if not affected
bool checkSatisfactionUnderAP(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                              std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                              std::unordered_set<int> &unallocated_UE_set,
                              std::vector<MyUeNode> my_UE_list,
                              int VLC_AP_index,
                              int UE_index)
{
    std::vector<std::vector<int>> resource_unit_matrix = resource_unit_matrix_per_VLC_AP[VLC_AP_index];

    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getID() < b.getID();});

    // at first, allocate resource to those unallocated UEs
    int subcarrier_idx = effective_subcarrier_num;
    int time_slot_idx = time_slot_num - 1;

    while (subcarrier_idx > 1 && resource_unit_matrix[subcarrier_idx][time_slot_idx] == 1) {
        time_slot_idx--;

        if (time_slot_idx == -1) {
            time_slot_idx = time_slot_num - 1;
            subcarrier_idx--;
        }
    }

    for (auto it = unallocated_UE_set.begin(); it != unallocated_UE_set.end(); it++) {
        double demand = my_UE_list[*it].getRequiredDataRate();
        int subcarrier_start = subcarrier_idx;
        int time_slot_start = time_slot_idx;

        // find the first subcarrier that provides data rate for UE *it
        while (VLC_data_rate_matrix[VLC_AP_index][*it][subcarrier_start] == 0.0) {
            subcarrier_start--;
            time_slot_start = time_slot_num - 1;
        }

        while (demand > 0 && subcarrier_start > 1) {
            if (resource_unit_matrix[subcarrier_start][time_slot_start] == 0) {
                demand -= VLC_data_rate_matrix[VLC_AP_index][*it][subcarrier_start];
                resource_unit_matrix[subcarrier_start][time_slot_start] = 1;
            }
            time_slot_start--;

            if (time_slot_start == -1) {
                time_slot_start = time_slot_num - 1;
                subcarrier_start--;
            }
        }

        if (demand > 0)
            return false;
    }

    // then check if this incoming UE can be satisfied
    double demand = my_UE_list[UE_index].getRequiredDataRate();
    int subcarrier_start = subcarrier_idx;
    int time_slot_start = time_slot_idx;

    // find the first subcarrier that provides data rate for UE *it
    while (VLC_data_rate_matrix[VLC_AP_index][UE_index][subcarrier_start] == 0.0) {
        subcarrier_start--;
        time_slot_start = time_slot_num - 1;
    }

    while (demand > 0 && subcarrier_start > 1) {
        if (resource_unit_matrix[subcarrier_start][time_slot_start] == 0) {
            demand -= VLC_data_rate_matrix[VLC_AP_index][UE_index][subcarrier_start];
            resource_unit_matrix[subcarrier_start][time_slot_start] = 1;
        }
        time_slot_start--;

        if (time_slot_start == -1) {
            time_slot_start = time_slot_num - 1;
            subcarrier_start--;
        }
    }

    if (demand > 0)
        return false;
    else
        return true;
}


// consider handover
void proposedMethodForStateN(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<int> &blocked_UE,
                             std::vector<MyUeNode> &my_UE_list)
{

}


// sort VLC APs in the descending order based on their residual resource after fulfilling UE's demand
std::vector<int> sortApBasedOnResidualResource(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                               std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                                               MyUeNode &UE)
{
    std::map<int,int,std::greater<int>> residual_RU_num; // {the number of residual RUs, VLC AP index}
    std::vector<double> offered_data_rate_per_AP;

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        std::vector<std::vector<int>> resource_unit_matrix = resource_unit_matrix_per_VLC_AP[VLC_AP_idx];
        double UE_demand = UE.getRequiredDataRate();
        double offered_data_rate = 0.0;
        int UE_idx = UE.getID();
        int subcarrier_idx = effective_subcarrier_num;
        int time_slot_idx = time_slot_num - 1;

        // find the first subcarrier that provides data rate for UE
        while (VLC_data_rate_matrix[VLC_AP_index][*it][subcarrier_idx] == 0.0)
            subcarrier_idx--;

        // find the beginning of the resource_unit_matrix
        while (subcarrier_idx > 1 && resource_unit_matrix[subcarrier_idx][time_slot_idx] == 1) {
            time_slot_idx--;

            if (time_slot_idx == -1) {
                time_slot_idx = time_slot_num - 1;
                subcarrier_idx--;
            }
        }

        // try to fulfill user's demand from high-freq subcarrier to low-freq
        while ((offered_data_rate < UE_demand) && (subcarrier_idx > 1)) {
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
        /*// this AP does not fulfill the user's demand, then set residual RU number to -1
        else
            residual_RU_num[-1] = VLC_AP_idx;
        */
    }


    std::vector<int> result;
    auto it = residual_RU_num.begin();

    // for those APs that fulfill the demand, sort them by the number of residual RUs,
    for (; it != residual_RU_num.end(); it++) {
        result.push_back(it->second);
    }

    /*// for those APs that don't fulfill the demand, sort them by data rate they offer
    std::map<double,int,std::greater<double>> sorting; // {data rate, VLC AP index}

    for (; it != residual_RU_num.end(); it++) {
        sorting[offered_data_rate_per_AP[it->second]] = it->second;
    }
    for (auto it2 = sorting.begin(); it2 != sorting.end(); it2++)
        result.push_back(it2->second);
    */

    return result;
}

