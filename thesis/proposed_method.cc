/*
    Note that
    (1) if an variable represents real index of something, like UE, AP,..., then it will be added with suffix "_idx".
*/

#include <map>
#include <limits>
#include <algorithm>
#include <unordered_set>

#include "print.h"
#include "channel.h"
#include "my_UE_node.h"
#include "proposed_method.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"


void proposedDynamicLB (int &state,
                           NodeContainer &RF_AP_node,
                           NodeContainer &VLC_AP_nodes,
                           NodeContainer &UE_nodes,
                           std::vector<std::vector<double>> &VLC_LOS_matrix,
                           std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                           std::vector<double> &RF_data_rate_vector,
                           std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                           std::vector<std::vector<int>> &AP_association_matrix,
                           std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                           std::vector<double> &demand_discount_per_AP,
                           std::vector<std::pair<int, int>> &first_empty_RU_position,
                           std::vector<double> &avg_satisfaction_per_AP,
                           std::vector<MyUeNode> &my_UE_list)
{
    std::vector<int> rejected_UE;

    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);

    if (state % complete_config_period == 0)
        completeConfiguration(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                                 rejected_UE, demand_discount_per_AP, first_empty_RU_position, avg_satisfaction_per_AP, my_UE_list);
    else
        partialConfiguration(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                                 rejected_UE, demand_discount_per_AP, first_empty_RU_position, avg_satisfaction_per_AP, my_UE_list);


#if DEBUG_MODE
    std::cout << "the number of UEs being rejected: " << rejected_UE.size() << std::endl;

    /*for (int i = 0; i < VLC_AP_num; i++) {
        printResourceUnitMatrix(RU_matrix_per_VLC_AP, i);
    }*/
#endif // DEBUG_MODE

}


void partialConfiguration(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                         std::vector<double> &RF_data_rate_vector,
                         std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                         std::vector<std::vector<int>> &AP_association_matrix,
                         std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                         std::vector<int> &rejected_UE,
                         std::vector<double> &demand_discount_per_AP,
                         std::vector<std::pair<int, int>> &first_empty_RU_position,
                         std::vector<double> &avg_satisfaction_per_AP,
                         std::vector<MyUeNode> &my_UE_list)
{
    //step1: construct UE demand vector and serving_UE vector for each AP
    double avg_satisfaction = 0.0;
    std::vector<double> UE_demand (UE_num, 0.0);
    std::vector<std::vector<int>> old_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());
    std::vector<std::vector<int>> new_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        avg_satisfaction += my_UE_list[UE_idx].getLastSatisfaction();
        old_serving_UE[my_UE_list[UE_idx].getCurrAssociatedAP()].push_back(UE_idx);
        UE_demand[UE_idx] = my_UE_list[UE_idx].getRequiredDataRate();
    }
    avg_satisfaction /= UE_num;

    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getRequiredDataRate() > b.getRequiredDataRate();});


    // step2: For each VLC AP allocate the least demand to UE who was connected to this AP in the previous state
    std::vector<std::pair<int, int>> first_empty_RU_position_copy (VLC_AP_num, std::make_pair(effective_subcarrier_num, time_slot_num-1));
    std::vector<std::vector<std::vector<int>>> local_RU_matrix (VLC_AP_num, std::vector<std::vector<int>> (effective_subcarrier_num + 1, std::vector<int> (time_slot_num, 0)));

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        for (int j = 0; j < old_serving_UE[VLC_AP_idx + RF_AP_num].size() && first_empty_RU_position_copy[VLC_AP_idx].first < 1; j++) {
            int UE_idx = old_serving_UE[VLC_AP_idx + RF_AP_num][j];
            int update_flag;
            int subcarrier_idx = first_empty_RU_position_copy[VLC_AP_idx].first;
            int time_slot_idx = first_empty_RU_position_copy[VLC_AP_idx].second;
            double discounted_demand = UE_demand[UE_idx] * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];

            // find the first subcarrier that provides data rate for UE *it
            update_flag = findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

            while (discounted_demand > 0 && subcarrier_idx >= 1) {
                if (local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                    discounted_demand -= VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                    local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                }
                goToNextRU(subcarrier_idx, time_slot_idx);
            }

            // since UEs can get at most their demand multiplied by demand discount, return one RU back
            if (discounted_demand < 0) {
                goToPrevRU(subcarrier_idx, time_slot_idx);
                local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
            }

            if (update_flag)
                first_empty_RU_position_copy[VLC_AP_idx] = std::make_pair(subcarrier_idx, time_slot_idx);
        }
    }

    // step3: adjust UEs whose satisfaction is below average
    std::vector<double> throughput (UE_num, 0.0);

    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();
        int prev_AP = my_UE_list[i].getCurrAssociatedAP(); // since we haven't updated APA results yet, the associated AP in the last state is still recorded in current AP
        double last_satisfaction = my_UE_list[i].getLastSatisfaction();

        // satisfaction is above average -> continue to use the current AP and the same RUs, so no need of RA
        if (last_satisfaction >= avg_satisfaction) {
            my_UE_list[i].setCurrAssociatedAP(prev_AP);
            AP_association_matrix[prev_AP][UE_idx] = 1; // optional
            new_serving_UE[prev_AP].push_back(UE_idx);

            throughput[UE_idx] = my_UE_list[i].getLastThroughput();
        }
        else {
            // find other VLC APs which can provide data rate and whose avg. satisfaction is larger and which still has available RU
            int chosen_AP = -1;
            double max_offered_data_rate = 0.0;

            for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
                if (VLC_AP_idx != prev_AP && VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] != 0.0 && avg_satisfaction_per_AP[RF_AP_num + VLC_AP_idx] > last_satisfaction
                    && first_empty_RU_position_copy[VLC_AP_idx].first >= 1)
                {
                    // step2-2-2: allocate resource to the current UE
                    int subcarrier_idx = first_empty_RU_position_copy[VLC_AP_idx].first;
                    int time_slot_idx = first_empty_RU_position_copy[VLC_AP_idx].second;
                    double offered_data_rate = 0.0;
                    double discounted_demand = my_UE_list[i].getRequiredDataRate() * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];

                    findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

                    while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
                        if (local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                            offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                            // local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                        }
                        goToNextRU(subcarrier_idx, time_slot_idx);
                    }

                    // since UEs can get at most their demand multiplied by demand discount, return one RU backs
                    if (offered_data_rate > discounted_demand) {
                        goToPrevRU(subcarrier_idx, time_slot_idx);
                        offered_data_rate -= VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                        // local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                    }

                    if (offered_data_rate > max_offered_data_rate) {
                        max_offered_data_rate = offered_data_rate;
                        chosen_AP = VLC_AP_idx + RF_AP_num;
                    }
                }
            }

            // determine which APs is best among RF AP, chosen VLC AP and previous AP
            if (max_offered_data_rate > RF_data_rate_vector[old_serving_UE[0].size()+1]) {
                if (max_offered_data_rate <= my_UE_list[i].getLastThroughput())
                    chosen_AP = my_UE_list[i].getCurrAssociatedAP();
            }
            else {
                if (my_UE_list[i].getLastThroughput() >= RF_data_rate_vector[old_serving_UE[0].size()+1])
                    chosen_AP = prev_AP;
                else
                    chosen_AP = 0;
            }

            AP_association_matrix[chosen_AP][UE_idx] = 1;
            my_UE_list[i].setCurrAssociatedAP(chosen_AP);
            new_serving_UE[chosen_AP].push_back(UE_idx);

            if (chosen_AP != prev_AP) { // new AP is not the same as the one in previous state
                AP_association_matrix[prev_AP][UE_idx] = 0;

                // release resource the UE had first
                if (prev_AP != 0)
                    releaseResource(RU_matrix_per_VLC_AP[prev_AP], first_empty_RU_position[prev_AP], my_UE_list[i]);

                // then allocate resource of the new AP to this UE
                if (chosen_AP != 0)
                    throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[chosen_AP][UE_idx], RU_matrix_per_VLC_AP[chosen_AP], first_empty_RU_position[chosen_AP],
                                                            max_offered_data_rate, my_UE_list[i]);
            }
        }
    }

    // UEs served by RF AP are not allocated resource until the entire APA+RA process is done
    for (int i = 0; i < new_serving_UE[0].size(); i++)
        throughput[new_serving_UE[0][i]] = RF_data_rate_vector[new_serving_UE[0].size()];


    // step3: RRA
    std::vector<double> satisfaction (UE_num, 0.0);
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getID() < b.getID();});

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        residualResourceAllocation(demand_discount_per_AP[VLC_AP_idx + RF_AP_num], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction,
                                   new_serving_UE[VLC_AP_idx + RF_AP_num], first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);
    }

    // step4: adjust demand discount ratio of each AP
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        int i = 0;
        for (; i < new_serving_UE[VLC_AP_idx + RF_AP_num].size(); i++) {
            int UE_idx = new_serving_UE[VLC_AP_idx + RF_AP_num][i];

            if (satisfaction[UE_idx] < demand_discount_per_AP[VLC_AP_idx + RF_AP_num]) {
                takeResourceBack(demand_discount_per_AP[VLC_AP_idx + RF_AP_num], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, new_serving_UE[VLC_AP_idx + RF_AP_num],
                                 first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);
                break;
            }
        }

        if (i == new_serving_UE[VLC_AP_idx + RF_AP_num].size()) {
            demand_discount_per_AP[VLC_AP_idx+RF_AP_num] = std::min(demand_discount_per_AP[VLC_AP_idx + RF_AP_num] + delta_p, 1.0);
        }
    }

    // finally, update throughput each UE gets in this state
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);

    // calculate average satisfaction in each AP
    calculateAvgSatisfactionForEachAP(new_serving_UE, satisfaction, avg_satisfaction_per_AP);
}


// rewrite
void completeConfiguration(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                         std::vector<double> &RF_data_rate_vector,
                         std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                         std::vector<std::vector<int>> &AP_association_matrix,
                         std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                         std::vector<int> &rejected_UE,
                         std::vector<double> &demand_discount_per_AP,
                         std::vector<std::pair<int, int>> &first_empty_RU_position,
                         std::vector<double> &avg_satisfaction_per_AP,
                         std::vector<MyUeNode> &my_UE_list)
{
    // pre-step: clear AP association matrix, RU matrix and first position of empty RU
    for (int i = 0; i < AP_association_matrix.size(); i++)
        for (int j = 0; j < AP_association_matrix[i].size(); j++)
            AP_association_matrix[i][j] = 0;

    for (int i = 0; i < RU_matrix_per_VLC_AP.size(); i++) {
        first_empty_RU_position[i] = std::make_pair(effective_subcarrier_num, time_slot_num-1);

        for (int j = 0; j < RU_matrix_per_VLC_AP[i].size(); j++)
            for (int k = 0; k < RU_matrix_per_VLC_AP[i][j].size(); k++)
                RU_matrix_per_VLC_AP[i][j][k] = 0;
    }

    // clear RU_block of each UE_node
    for (int i = 0; i < my_UE_list.size(); i++) {
        my_UE_list[i].clearRuBlock();
    }


    // step1: find the AP with the best SINR for each UE
    std::vector<int> served_by_RF;
    std::vector<int> best_SINR_AP (UE_num, -1);
    std::vector<std::unordered_set<int>> unallocated_UE_under_best_VLC_AP(VLC_AP_num, std::unordered_set<int> ());
    findBestSinrAP(best_SINR_AP, unallocated_UE_under_best_VLC_AP, served_by_RF, VLC_SINR_matrix);


    // step2: record user's demand in another space and then sort users in the descending order based on their demands
    std::vector<double> UE_demand;
    for (int i = 0; i < UE_num; i++)
        UE_demand.push_back(my_UE_list[i].getRequiredDataRate());

    // note that be careful of the order of my_UE_list from now on
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getRequiredDataRate() > b.getRequiredDataRate();});


    // step3: APA + RA
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<double> satisfaction (UE_num, 0.0);
    std::vector<std::vector<int>> serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();

        if (best_SINR_AP[UE_idx] != 0) {
            /* APA */
            unallocated_UE_under_best_VLC_AP[best_SINR_AP[UE_idx] - RF_AP_num].erase(UE_idx);

            // <chosen AP, offered data rate> is returned
            std::pair<int, double> result = accessPointAssociation(VLC_data_rate_matrix, RF_data_rate_vector, served_by_RF, RU_matrix_per_VLC_AP, unallocated_UE_under_best_VLC_AP,
                                                                    UE_demand, demand_discount_per_AP, first_empty_RU_position, my_UE_list[i]);
            int chosen_AP = result.first;

            if (chosen_AP < 0) { // no serving AP
                rejected_UE.push_back(UE_idx);
                my_UE_list[i].setCurrAssociatedAP(chosen_AP);
            }
            else {
                // update APA result
                AP_association_matrix[chosen_AP][UE_idx] = 1;
                my_UE_list[i].setCurrAssociatedAP(chosen_AP);
                serving_UE[chosen_AP].push_back(UE_idx);

                /* VLC RA */
                if (chosen_AP > 0) {
                    throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[chosen_AP - RF_AP_num][UE_idx], RU_matrix_per_VLC_AP[chosen_AP - RF_AP_num],
                                                            first_empty_RU_position[chosen_AP - RF_AP_num], result.second, my_UE_list[i]);

                    satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
                }

            }
        }
        else {
            int chosen_AP = best_SINR_AP[UE_idx];  // should be 0

            AP_association_matrix[chosen_AP][UE_idx] = 1;
            my_UE_list[i].setCurrAssociatedAP(chosen_AP);
            serving_UE[chosen_AP].push_back(UE_idx);
        }
    }

    // UEs served by RF AP are not allocated resource until the entire APA+RA process is done
    for (int i = 0; i < served_by_RF.size(); i++) {
        throughput[served_by_RF[i]] = RF_data_rate_vector[served_by_RF.size()];
        satisfaction[served_by_RF[i]] = std::min(throughput[served_by_RF[i]] / UE_demand[served_by_RF[i]], 1.0);
    }


    //step4: RRA
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getID() < b.getID();});

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        residualResourceAllocation(demand_discount_per_AP[VLC_AP_idx + RF_AP_num], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, serving_UE[VLC_AP_idx + RF_AP_num],
                                   first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);
    }


    // finally, update throughput each UE gets in this state
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);

    // calculate average satisfaction in each AP
    calculateAvgSatisfactionForEachAP(serving_UE, satisfaction, avg_satisfaction_per_AP);
}

void findBestSinrAP(std::vector<int> &best_SINR_AP,
                    std::vector<std::unordered_set<int>> &unallocated_UE_under_best_VLC_AP,
                    std::vector<int> &served_by_RF,
                    std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix)
{
    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        int best_VLC_AP = -1;
        std::vector<double> best_SINR_vec (effective_subcarrier_num+1, -1);

        for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
            for (int subcarrier_idx = 1; subcarrier_idx <= effective_subcarrier_num; subcarrier_idx++) {
                if (VLC_SINR_matrix[VLC_AP_idx][UE_idx][subcarrier_idx] == best_SINR_vec[subcarrier_idx])
                    continue;
                else if (VLC_SINR_matrix[VLC_AP_idx][UE_idx][subcarrier_idx] > best_SINR_vec[subcarrier_idx]) {
                    best_SINR_vec = VLC_SINR_matrix[VLC_AP_idx][UE_idx];
                    best_VLC_AP = VLC_AP_idx;
                }
                break;
            }
        }

        // no VLC AP can provide data rate for this UE, so choose RF AP as its best AP
        if (best_VLC_AP == -1 || best_SINR_vec[1] < 1) {
            best_SINR_AP[UE_idx] = 0;
            served_by_RF.push_back(UE_idx);
        }
        else {
            best_SINR_AP[UE_idx] = best_VLC_AP + RF_AP_num;
            unallocated_UE_under_best_VLC_AP[best_VLC_AP].insert(UE_idx);
        }
    }
}

std::pair<int, double> accessPointAssociation(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                                std::vector<double> &RF_data_rate_vector,
                                                std::vector<int> &served_by_RF,
                                                std::vector<std::vector<std::vector<int>>> RU_matrix_per_VLC_AP,
                                                std::vector<std::unordered_set<int>> &unallocated_UE_under_best_VLC_AP,
                                                std::vector<double> &UE_demand,
                                                std::vector<double> &demand_discount_per_AP,
                                                std::vector<std::pair<int, int>> first_empty_RU_position,
                                                MyUeNode &UE_node)
{
    int UE_idx = UE_node.getID();
    std::vector<std::pair<int, double>> VLC_AP_pair; // record <number of residual RUs, offered data rate>

    // step1: for each VLC AP allocate resource to those unallocated UEs first, and then the current UE
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] == 0.0) {
            VLC_AP_pair.emplace_back(0, 0.0);
            continue;
        }

        int skip_flag = 0;
        // step1-1: allocate resource to those unallocated UEs
        for (auto it = unallocated_UE_under_best_VLC_AP[VLC_AP_idx].begin(); it != unallocated_UE_under_best_VLC_AP[VLC_AP_idx].end(); it++) {
            int update_flag;
            int subcarrier_idx = first_empty_RU_position[VLC_AP_idx].first;
            int time_slot_idx = first_empty_RU_position[VLC_AP_idx].second;
            double discounted_demand = UE_demand[*it] * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];

            // find the first subcarrier that provides data rate for UE *it
            update_flag = findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][*it], subcarrier_idx, time_slot_idx);

            while (discounted_demand > 0 && subcarrier_idx >= 1) {
                if (RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                    discounted_demand -= VLC_data_rate_matrix[VLC_AP_idx][*it][subcarrier_idx];
                    RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                }
                goToNextRU(subcarrier_idx, time_slot_idx);
            }

            /*// since UEs can get at most their demand multiplied by demand discount, return one RU back
            if (discounted_demand < 0) {
                goToPrevRU(subcarrier_idx, time_slot_idx);
                R_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
            }*/

            if (update_flag)
                first_empty_RU_position[VLC_AP_idx] = std::make_pair(subcarrier_idx, time_slot_idx);

            // AP has no residual resource
            if (first_empty_RU_position[VLC_AP_idx].first < 1) {
                VLC_AP_pair.emplace_back(0, 0.0);
                skip_flag = 1;
                break;
            }
        }

        // step1-2: allocate resource to the current UE
        if (!skip_flag) {
            int subcarrier_idx = first_empty_RU_position[VLC_AP_idx].first;
            int time_slot_idx = first_empty_RU_position[VLC_AP_idx].second;
            double offered_data_rate = 0.0;
            double discounted_demand = UE_node.getRequiredDataRate() * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];

            findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

            while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
                if (RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                    offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                    RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
                }
                goToNextRU(subcarrier_idx, time_slot_idx);
            }

            /*// since UEs can get at most their demand multiplied by demand discount, return one RU backs
            if (offered_data_rate > discounted_demand) {
                goToPrevRU(subcarrier_idx, time_slot_idx);
                offered_data_rate -= VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
            }*/

            int empty_RU_num = 0;
            for (int i = 1; i <= effective_subcarrier_num; i++) {
                for (int j = 0; j < time_slot_num; j++) {
                    if (RU_matrix_per_VLC_AP[VLC_AP_idx][i][j] == 0)
                        empty_RU_num++;
                }
            }

            VLC_AP_pair.emplace_back(empty_RU_num, offered_data_rate);
        }
    }

    // step2: after all VLC APs finished allocation, choose the one with max offered data rate and residual RUs
    int result_VLC_AP = -1;
    std::pair<int, double> max_value = std::make_pair(-1, -1);

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_pair.size(); VLC_AP_idx++) {
        if (VLC_AP_pair[VLC_AP_idx].second == 0.0) // skip those VLC APs which provide no data rate
            continue;

        if (VLC_AP_pair[VLC_AP_idx].second > max_value.second) {
            max_value = VLC_AP_pair[VLC_AP_idx];
            result_VLC_AP = VLC_AP_idx;
        }
        else if (VLC_AP_pair[VLC_AP_idx].second == max_value.second) {
            if (VLC_AP_pair[VLC_AP_idx].first > max_value.first) {
                max_value = VLC_AP_pair[VLC_AP_idx];
                result_VLC_AP = VLC_AP_idx;
            }
        }
    }

    //&& (VLC_AP_pair[result_VLC_AP].second >= UE_node.getRequiredDataRate() * demand_discount_per_AP[RF_AP_num + result_VLC_AP])
    if (max_value.second > 0.0) {
        return std::make_pair(result_VLC_AP + RF_AP_num, VLC_AP_pair[result_VLC_AP].second);
    }


    // step3: if no VLC AP can serve this UE, then try RF AP
    int i = 0;
    for (; i < served_by_RF.size(); i++) {
        if (UE_demand[served_by_RF[i]] * demand_discount_per_AP[0] > RF_data_rate_vector[served_by_RF.size()])
            break;
    }

    if (i == served_by_RF.size() && UE_demand[UE_idx] * demand_discount_per_AP[0] <= RF_data_rate_vector[served_by_RF.size()+1]) {
        served_by_RF.push_back(UE_idx);
        return std::make_pair(0, RF_data_rate_vector[served_by_RF.size()+1]);
    }

    return std::make_pair(-1, 0.0);
}

double resourceAllocation(std::vector<double> &VLC_data_rate_matrix,
                            std::vector<std::vector<int>> &RU_matrix,
                            std::pair<int, int> &first_empty_RU_position,
                            double offered_data_rate,
                            MyUeNode &UE_node)
{
    if (offered_data_rate <= 0.0)
        return 0.0;

    int update_flag;
    int subcarrier_idx = first_empty_RU_position.first;
    int time_slot_idx = first_empty_RU_position.second;
    double throughput = 0.0;
    std::pair<int, int> start;

    update_flag = findFirstEffectiveSubcarrier(VLC_data_rate_matrix, subcarrier_idx, time_slot_idx);

    // find the first available RU
    while (subcarrier_idx >=1 && RU_matrix[subcarrier_idx][time_slot_idx] == 1)
        goToNextRU(subcarrier_idx, time_slot_idx);

    start = std::make_pair(subcarrier_idx, time_slot_idx);

    while (offered_data_rate > 0 && subcarrier_idx >= 1) {
        if (RU_matrix[subcarrier_idx][time_slot_idx] == 0) {
            offered_data_rate -= VLC_data_rate_matrix[subcarrier_idx];
            throughput += VLC_data_rate_matrix[subcarrier_idx];
            RU_matrix[subcarrier_idx][time_slot_idx] = 1;

            goToNextRU(subcarrier_idx, time_slot_idx);
        }
        // reach an RU that has been used, so record the range of RU this UE uses so far and push back to its RU_used vector
        else {
            std::pair<int, int> tail;
            tail = (time_slot_idx == time_slot_num - 1) ? std::make_pair(subcarrier_idx + 1, 0) : std::make_pair(subcarrier_idx, time_slot_idx + 1);
            UE_node.recordResourceUnit(start, tail);

            // find the first available RU
            while (subcarrier_idx >= 1 && RU_matrix[subcarrier_idx][time_slot_idx] == 1)
                goToNextRU(subcarrier_idx, time_slot_idx);

            start = std::make_pair(subcarrier_idx, time_slot_idx);
        }
    }

    /*// since UEs can get at most their demand multiplied by demand discount, return one RU back
    if (offered_data_rate < 0) {
        goToPrevRU(subcarrier_idx, time_slot_idx);
        RU_matrix[subcarrier_idx][time_slot_idx] = 0;
        offered_data_rate -= VLC_data_rate_matrix[subcarrier_idx];
    }*/

    if (update_flag)
        first_empty_RU_position = std::make_pair(subcarrier_idx, time_slot_idx);

    if (start.first != subcarrier_idx || start.second != time_slot_idx) {
        goToPrevRU(subcarrier_idx, time_slot_idx); // we have to go back one RU, since the pointer stops at one RU ahead of the last RU this UE uses
        UE_node.recordResourceUnit(start, std::make_pair(subcarrier_idx, time_slot_idx));
    }

    return throughput;
}


void residualResourceAllocation(double &discount_ratio,
                                std::vector<std::vector<double>> &VLC_data_rate_matrix,
                                std::vector<double> &throughput,
                                std::vector<double> &satisfaction,
                                std::vector<int> &serving_UE,
                                std::pair<int, int> &first_empty_RU_position,
                                std::vector<std::vector<int>> &RU_matrix,
                                std::vector<MyUeNode> &my_UE_list)
{
    if (first_empty_RU_position.first < 1 || serving_UE.empty())
        return;

    // step1: allocate residual resource to those users whose satisfaction is below discount ratio
    std::vector<int> below, above;

    for (int i = 0; i < serving_UE.size(); i++) {
        if (satisfaction[serving_UE[i]] < discount_ratio)
            below.push_back(serving_UE[i]);
    }

    makeUpResourceDifference(discount_ratio, VLC_data_rate_matrix, throughput, satisfaction, below, first_empty_RU_position, RU_matrix, my_UE_list);


    // step2: see whether every user's satisfaction is all above discount ratio
    int index = 0;
    for (; index < serving_UE.size(); index++) {
        if (satisfaction[serving_UE[index]] < discount_ratio)
            break;
    }

    // if there exist a user's satisfaction is below discount ratio => decrease discount ratio, take resource back from those whose satisfaction is higher than discount ratio
    // and allocate returned resource to "below" users
    if (index != serving_UE.size()) {
        discount_ratio = std::max(discount_ratio - delta_p, min_discount_ratio);

        for (int i = 0; i < serving_UE.size(); i++) {
            if (satisfaction[serving_UE[i]] > discount_ratio)
                above.push_back(serving_UE[i]);
        }

        takeResourceBack(discount_ratio, VLC_data_rate_matrix, throughput, satisfaction, above, first_empty_RU_position, RU_matrix, my_UE_list);
        makeUpResourceDifference(discount_ratio, VLC_data_rate_matrix, throughput, satisfaction, below, first_empty_RU_position, RU_matrix, my_UE_list);
    }
    else
        discount_ratio = std::min(discount_ratio + delta_p, 1.0);

    // if still resource not being allocated, then allocate them to users equally
    if (first_empty_RU_position.first >= 1)
        allocateResourceEqually(VLC_data_rate_matrix, throughput, satisfaction, serving_UE, first_empty_RU_position, RU_matrix, my_UE_list);
}

void takeResourceBack(double discount_ratio,
                        std::vector<std::vector<double>> &VLC_data_rate_matrix,
                        std::vector<double> &throughput,
                        std::vector<double> &satisfaction,
                        std::vector<int> &target_UE,
                        std::pair<int, int> &first_empty_RU_position,
                        std::vector<std::vector<int>> &RU_matrix,
                        std::vector<MyUeNode> &my_UE_list)
{
    // step1: take back resource allocated to those UE in above vector from low freq to high freq
    for (int i = 0; i < target_UE.size(); i++) {
        int UE_idx = target_UE[i];
        double resource_return = (satisfaction[UE_idx] - discount_ratio) * my_UE_list[UE_idx].getRequiredDataRate(); // the amount of resource has to return

        my_UE_list[UE_idx].arrangeRuBlock(); // make sure that RU blocks are ordered in the descending order

        int RU_block_idx = my_UE_list[UE_idx].getRuBlockSize() - 1;
        while (resource_return > 0 && RU_block_idx >= 0) {
            RuRangeType range = my_UE_list[UE_idx].getNthResourceUnitBlock(RU_block_idx);
            std::pair<int, int> start = range.second;
            std::pair<int, int> tail = range.first;

            while (resource_return > 0 && (start.first < tail.first || (start.first == tail.first && start.second <= tail.second))) {
                resource_return -= VLC_data_rate_matrix[UE_idx][start.first];
                throughput[UE_idx] -= VLC_data_rate_matrix[UE_idx][start.first];

                RU_matrix[start.first][start.second] = 0;

                goToPrevRU(start.first, start.second);
            }

            goToNextRU(start.first, start.second); // make "start" point to the last released RU
            if (first_empty_RU_position.first < start.first || (first_empty_RU_position.first == start.first && first_empty_RU_position.second < start.second))
                    first_empty_RU_position = start;

            if (start.first == tail.first && start.second == tail.second)
                my_UE_list[UE_idx].removeLastResourceUnitBlock();
            else {
                goToPrevRU(start.first, start.second);
                my_UE_list[UE_idx].updateNthResourceUnitBlock(RU_block_idx, std::make_pair(start, tail));
            }

            RU_block_idx--;
        }

        satisfaction[UE_idx] = std::min(throughput[UE_idx] / my_UE_list[UE_idx].getRequiredDataRate(), 1.0);
    }
}

void releaseResource(std::vector<std::vector<int>> &RU_matrix,
                      std::pair<int, int> &first_empty_RU_position,
                      MyUeNode &UE_node)
{
    std::vector<RuRangeType> RU_block = UE_node.getWholeRuBlock();

    for (int i = 0; i < RU_block.size(); i++) {
        std::pair<int, int> head = RU_block[i].first;
        std::pair<int, int> tail = RU_block[i].second;

        while (head.first > tail.first || (head.first == tail.first && head.second >= tail.second)) {
            RU_matrix[head.first][head.second] = 1;

            if (first_empty_RU_position.first < head.first || (first_empty_RU_position.first == head.first && first_empty_RU_position.second < head.second))
                first_empty_RU_position = head;

            goToNextRU(head.first, head.second);
        }
    }

    UE_node.clearRuBlock();
}


void updateInfoToMyUeList(std::vector<double> &throughput,
                          std::vector<double> &satisfaction,
                          std::vector<MyUeNode> &my_UE_list)
{
    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        my_UE_list[UE_idx].addThroughput(throughput[UE_idx]);
        my_UE_list[UE_idx].addSatisfaction(satisfaction[UE_idx]);
    }
}

void makeUpResourceDifference(double discount_ratio,
                                std::vector<std::vector<double>> &VLC_data_rate_matrix,
                                std::vector<double> &throughput,
                                std::vector<double> &satisfaction,
                                std::vector<int> &target_UE,
                                std::pair<int, int> &first_empty_RU_position,
                                std::vector<std::vector<int>> &RU_matrix,
                                std::vector<MyUeNode> &my_UE_list)
{
    if (first_empty_RU_position.first < 1 || target_UE.empty())
        return;

    // step1: sort target_UE based on the amount of resource having to make up
    std::vector<std::pair<int, double>> vec; // <UE_idx, demand - throughput>
    vec.reserve(target_UE.size());

    for (int i = 0; i < target_UE.size(); i++) {
        int UE_idx = target_UE[i];
        double diff = my_UE_list[UE_idx].getRequiredDataRate() * discount_ratio - throughput[UE_idx];

        vec.emplace_back(UE_idx, diff);
    }

    std::sort(vec.begin(), vec.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b){return a.second >= b.second;});

    // step2: allocate residual resource to make UEs have minimum discounted data rate
    int index = 0;
    while (first_empty_RU_position.first >= 1 && index < vec.size() && vec[index].second > 0) {
        int UE_idx = vec[index].first;
        double diff = vec[index].second;

        throughput[UE_idx] += resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, diff, my_UE_list[UE_idx]);
        index++;
    }

    updateSatisfaction(target_UE, throughput, satisfaction, my_UE_list);
}

// if there is still resource not being allocated, calculate how much resource is left (based on the UE with the best data rate),
// equally divide residual resource and allocate to each user from low-SINR user to high-SINR user
void allocateResourceEqually(std::vector<std::vector<double>> &VLC_data_rate_matrix,
                                std::vector<double> &throughput,
                                std::vector<double> &satisfaction,
                                std::vector<int> &serving_UE,
                                std::pair<int, int> &first_empty_RU_position,
                                std::vector<std::vector<int>> &RU_matrix,
                                std::vector<MyUeNode> &my_UE_list)
{
    if (first_empty_RU_position.first < 1 || serving_UE.empty())
        return;

    std::sort(serving_UE.begin(), serving_UE.end(), [&](const int UE_a, const int UE_b){return VLC_data_rate_matrix[UE_a][1] < VLC_data_rate_matrix[UE_b][1];});

    int best_UE = serving_UE.back();
    double total_additional_resource = 0.0;

    for (int i = 1; i < effective_subcarrier_num + 1; i++) {
        for (int j = 0; j < time_slot_num; j++) {
            total_additional_resource += (1 - RU_matrix[i][j]) * VLC_data_rate_matrix[best_UE][i];
        }
    }

    double estimated_additional_gained = total_additional_resource / serving_UE.size();
    for (int i = 0; i < serving_UE.size(); i++) {
        int UE_idx = serving_UE[i];
        double actual_additional_gained = resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, estimated_additional_gained, my_UE_list[UE_idx]);;

        throughput[UE_idx] += actual_additional_gained;

        // resource that the next user would have is the average of total_additional_resource over the rest of users
        total_additional_resource -= actual_additional_gained;

        if (i != serving_UE.size()-1)
            estimated_additional_gained = total_additional_resource / (serving_UE.size()-i-1);
    }

    updateSatisfaction(serving_UE, throughput, satisfaction, my_UE_list);
}

void updateSatisfaction(std::vector<int> &serving_UE, std::vector<double> &throughput, std::vector<double> &satisfaction, std::vector<MyUeNode> &my_UE_list)
{
    for (int i = 0; i < serving_UE.size(); i++) {
        int UE_idx = serving_UE[i];

        satisfaction[UE_idx] = std::min(throughput[UE_idx] / my_UE_list[UE_idx].getRequiredDataRate(), 1.0);
    }
}

void calculateAvgSatisfactionForEachAP(std::vector<std::vector<int>> &serving_UE, std::vector<double> &satisfaction, std::vector<double> &avg_satisfaction_per_AP)
{
    for (int AP_idx = 0; AP_idx < RF_AP_num + VLC_AP_num; AP_idx++) {
        double average = 0.0;

        for (int i = 0; i < serving_UE[AP_idx].size(); i++) {
            average += satisfaction[serving_UE[AP_idx][i]];
        }
        average /= serving_UE[AP_idx].size();

        avg_satisfaction_per_AP[AP_idx] = average;
    }
}


// find the first subcarrier that provides data rate for the specific UE
// if this UE does not use the first empty RU, then it's unnecessary for first_empty_RU_position to update
int findFirstEffectiveSubcarrier(std::vector<double> &VLC_data_rate_matrix, int &subcarrier_idx, int &time_slot_idx)
{
    int update_flag = 1;
    while (subcarrier_idx >= 1 && VLC_data_rate_matrix[subcarrier_idx] == 0.0) {
        update_flag = 0;
        subcarrier_idx--;
        time_slot_idx = time_slot_num - 1;
    }

    return update_flag;
}

void goToPrevRU(int &subcarrier_idx, int &time_slot_idx)
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
