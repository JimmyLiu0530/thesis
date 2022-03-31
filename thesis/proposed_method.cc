/*
    Note that
    (1) if an variable represents real index of something, like UE, AP,..., then it will be added with suffix "_idx".
*/

#include <map>
#include <limits>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include <math.h>

#include "print.h"
#include "channel.h"
#include "my_UE_node.h"
#include "proposed_method.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"



void proposedDynamicLB(int state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                       std::vector<std::vector<int>> &AP_association_matrix,
                       std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                       std::vector<double> &discount_ratio_per_AP,
                       std::vector<std::pair<int, int>> &first_empty_RU_position,
                       std::vector<MyUeNode> &my_UE_list)
{
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);

#if DEBUG_MODE
    printUePosition(my_UE_list);
#endif // DEBUG_MODE

    if (state % complete_config_period == 0)
        fullConfiguration(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                          discount_ratio_per_AP, first_empty_RU_position, my_UE_list);
    else
        partialConfiguration(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                            discount_ratio_per_AP, first_empty_RU_position, my_UE_list);


#if DEBUG_MODE
    for (int i = 0; i < VLC_AP_num; i++) {
        printResourceUnitMatrix(RU_matrix_per_VLC_AP, i);
    }
#endif // DEBUG_MODE

}

void partialConfiguration(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                         std::vector<double> &RF_data_rate_vector,
                         std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                         std::vector<std::vector<int>> &AP_association_matrix,
                         std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                         std::vector<double> &discount_ratio_per_AP,
                         std::vector<std::pair<int, int>> &first_empty_RU_position,
                         std::vector<MyUeNode> &my_UE_list)
{
    //step1: calculate average satisfaction of the previous state.
    double avg_satisfaction = 0.0;
    std::vector<double> UE_demand (UE_num, 0.0);
    std::vector<std::vector<int>> old_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());
    std::vector<std::vector<int>> new_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int UE_idx = 0; UE_idx < my_UE_list.size(); UE_idx++) {
        avg_satisfaction += my_UE_list[UE_idx].getLastSatisfaction();
        UE_demand[UE_idx] = my_UE_list[UE_idx].getRequiredDataRate();

        if (my_UE_list[UE_idx].getCurrAssociatedAP() >= 0)
            old_serving_UE[my_UE_list[UE_idx].getCurrAssociatedAP()].push_back(UE_idx);
    }
    avg_satisfaction /= UE_num;


    // step2: divide APs into two groups
    std::vector<int> higher_than_avg;
    std::vector<int> lower_than_avg;
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<double> satisfaction (UE_num, 0.0);

    for (int i = 0; i < my_UE_list.size(); i++) {
        throughput[my_UE_list[i].getID()] = my_UE_list[i].getLastThroughput();
        satisfaction[my_UE_list[i].getID()] = my_UE_list[i].getLastSatisfaction();
    }

    for (int AP_idx = 1; AP_idx < VLC_AP_num + RF_AP_num; AP_idx++) {
        if (discount_ratio_per_AP[AP_idx] > avg_satisfaction)
            higher_than_avg.push_back(AP_idx);
        else if (discount_ratio_per_AP[AP_idx] < avg_satisfaction)
            lower_than_avg.push_back(AP_idx);
    }

    // step2-1: for each AP in the "higher" group
    for (int i = 0; i < higher_than_avg.size(); i++) {
        // step2-1-1: further divide UEs served by this AP into two groups, “beyond avg. satisfaction” and “below avg. satisfaction”
        int AP_idx = higher_than_avg[i];
        std::vector<int> beyond, below;

        for (int j = 0; j < old_serving_UE[AP_idx].size(); j++) {
            int UE_idx = old_serving_UE[AP_idx][i];

            if (my_UE_list[UE_idx].getLastSatisfaction() > avg_satisfaction)
                beyond.push_back(UE_idx);
            else if (my_UE_list[UE_idx].getLastSatisfaction() < avg_satisfaction)
                below.push_back(UE_idx);
        }

        // step2-1-2: for UEs in the beyond group, take resource back from them until their satisfaction meet avg. satisfaction
        for (int j = 0; j < beyond.size(); j++) {
            int UE_idx = beyond[j];
            double resource_return = throughput[UE_idx] - avg_satisfaction * UE_demand[UE_idx];

            takeResourceBack(VLC_data_rate_matrix[AP_idx - RF_AP_num][UE_idx], first_empty_RU_position[AP_idx - RF_AP_num], RU_matrix_per_VLC_AP[AP_idx - RF_AP_num],
                             resource_return, throughput[UE_idx], my_UE_list[UE_idx]);
        }

        // step2-1-3: for UEs in the below avg. satisfaction group, allocate resource to them
        // in a manner that the UE reaching the avg. satisfaction with the least data rate is allocated first.
        std::sort(below.begin(), below.end(),
                  [&](const int UE_a, const int UE_b){ return ((avg_satisfaction - my_UE_list[UE_a].getLastSatisfaction()) * UE_demand[UE_a]) <
                                                                ((avg_satisfaction - my_UE_list[UE_b].getLastSatisfaction()) * UE_demand[UE_b]); });

        for (int j = 0; j < below.size(); j++) {
            int UE_idx = below[j];
            double extra_data_rate = avg_satisfaction * UE_demand[UE_idx] - throughput[UE_idx] ;

            throughput[UE_idx] += resourceAllocation(VLC_data_rate_matrix[AP_idx - RF_AP_num][UE_idx], RU_matrix_per_VLC_AP[AP_idx - RF_AP_num],
                                                     first_empty_RU_position[AP_idx - RF_AP_num], extra_data_rate, my_UE_list[UE_idx]);
        }
    }

    // step2-2: for each AP in the "lower" group
    for (int i = 0; i < lower_than_avg.size(); i++) {
        // step2-2-1: sort UEs served by this AP in the ascending order of satisfaction first
        int AP_idx = lower_than_avg[i];

        std::sort(old_serving_UE[AP_idx].begin(), old_serving_UE[AP_idx].end(),
                  [&](const int UE_a, const int UE_b){ return my_UE_list[UE_a].getLastSatisfaction() < my_UE_list[UE_b].getLastSatisfaction(); });

        // step2-2-2: take resource back from the first expel_ratio % of UEs
        int expelled_UE_num = floor(expel_ratio * old_serving_UE[AP_idx].size());

        for (int j = 0 ; j < expelled_UE_num; j++) {
            int UE_idx = old_serving_UE[AP_idx][j];

            throughput[UE_idx] = 0.0;
            releaseAllResource(RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], first_empty_RU_position[AP_idx - RF_AP_num], my_UE_list[UE_idx]);
        }

        // step2-2-3: allocate resource to UEs in a manner that the UE reaching the avg. satisfaction with the least data rate is allocated first
        std::sort(old_serving_UE[AP_idx].begin() + expelled_UE_num, old_serving_UE[AP_idx].end(),
                  [&](const int UE_a, const int UE_b){ return ((avg_satisfaction - my_UE_list[UE_a].getLastSatisfaction()) * UE_demand[UE_a]) <
                                                                ((avg_satisfaction - my_UE_list[UE_b].getLastSatisfaction()) * UE_demand[UE_b]); });
        for (int j = expelled_UE_num; j < old_serving_UE[AP_idx].size(); j++) {
            int UE_idx = old_serving_UE[AP_idx][j];
            double extra_data_rate = avg_satisfaction * UE_demand[UE_idx] - throughput[UE_idx];

            throughput[UE_idx] += resourceAllocation(VLC_data_rate_matrix[AP_idx - RF_AP_num][UE_idx], RU_matrix_per_VLC_AP[AP_idx - RF_AP_num],
                                                     first_empty_RU_position[AP_idx - RF_AP_num], extra_data_rate, my_UE_list[UE_idx]);
        }
    }

    // update satisfaction
    for (int i = 0; i < UE_num; i++) {
        satisfaction[i] = std::min(throughput[i] / UE_demand[i], 1.0);
    }


    // step3
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getRequiredDataRate() > b.getRequiredDataRate(); });


    // step4: APA+RA
    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();
        int prev_AP = my_UE_list[i].getCurrAssociatedAP(); // since we haven't updated APA results yet, the associated AP in the last state is still recorded in current AP

        // satisfaction is above average -> continue to use the current AP and the same RUs, so no need of RA
        if (satisfaction[UE_idx] >= avg_satisfaction) {
            my_UE_list[i].setCurrAssociatedAP(prev_AP);
            AP_association_matrix[prev_AP][UE_idx] = 1; // optional
            new_serving_UE[prev_AP].push_back(UE_idx);
        }
        else {
            // step3-2-1: find other VLC APs which can provide data rate and which still has available RU
            std::vector<std::pair<int, double>> candidate_VLC_AP;

            for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
                if (VLC_AP_idx != prev_AP - RF_AP_num && VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] > 0.0 && first_empty_RU_position[VLC_AP_idx].first >= 1) {
                    int subcarrier_idx = first_empty_RU_position[VLC_AP_idx].first;
                    int time_slot_idx = first_empty_RU_position[VLC_AP_idx].second;
                    double offered_data_rate = 0.0;
                    double discounted_demand = avg_satisfaction * UE_demand[UE_idx]; //discount_ratio_per_AP[RF_AP_num + VLC_AP_idx];

                    findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

                    while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
                        if (RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] == -1)
                            offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];

                        goToNextRU(subcarrier_idx, time_slot_idx);
                    }

                    if (offered_data_rate > throughput[UE_idx])
                        candidate_VLC_AP.emplace_back(VLC_AP_idx + RF_AP_num, offered_data_rate);
                }
            }

            if (prev_AP > 0) {
                if (RF_data_rate_vector[old_serving_UE[0].size() + 1] > throughput[UE_idx])
                    candidate_VLC_AP.emplace_back(0, RF_data_rate_vector[old_serving_UE[0].size() + 1]);
            }

            // if the candidate set is not empty, choose the AP with the most data rate from it.
            int new_AP = prev_AP;
            double offered_data_rate = 0.0;

            if (!candidate_VLC_AP.empty()) {
                int best_index = 0;
                for (int i = 1; i < candidate_VLC_AP.size(); i++) {
                    best_index = (candidate_VLC_AP[i].second > candidate_VLC_AP[best_index].second) ? i : best_index;
                }
                new_AP = candidate_VLC_AP[best_index].first;
                offered_data_rate = candidate_VLC_AP[best_index].second;
            }

            // step3-2-2: update APA and allocate resource to this UE
            AP_association_matrix[new_AP][UE_idx] = 1;
            new_serving_UE[new_AP].push_back(UE_idx);
            my_UE_list[i].setCurrAssociatedAP(new_AP);

            // if this new AP is not the prev AP, then we have to release resource the UE had first
            // and then allocate resource of the new AP to this UE and update the corresponding local RU matrix
            if (new_AP != prev_AP) {
                // release resource
                AP_association_matrix[prev_AP][UE_idx] = 0;

                if (prev_AP > 0)
                    releaseAllResource(RU_matrix_per_VLC_AP[prev_AP - RF_AP_num], first_empty_RU_position[prev_AP - RF_AP_num], my_UE_list[i]);

                else if (prev_AP == 0) {
                    std::vector<int>::iterator loc = old_serving_UE[0].begin();
                    for (; loc != old_serving_UE[0].end() && *loc != UE_idx; loc++);

                    if (loc != old_serving_UE[0].end())
                        old_serving_UE[0].erase(loc);
                }

                // allocate resource
                if (new_AP > 0) {
                    int new_VLC_AP = new_AP - RF_AP_num;
                    throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[new_VLC_AP][UE_idx], RU_matrix_per_VLC_AP[new_VLC_AP], first_empty_RU_position[new_VLC_AP],
                                                            offered_data_rate, my_UE_list[i]);

                    satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
                }
            }
        }
    }

    // UEs served by RF AP are not allocated resource until the entire APA+RA process is done
    for (int i = 0; i < new_serving_UE[0].size(); i++) {
        int UE_idx = new_serving_UE[0][i];

        throughput[UE_idx] = RF_data_rate_vector[new_serving_UE[0].size()];
        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }


    // step5: RRA
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getID() < b.getID(); });
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        residualResourceAllocation(discount_ratio_per_AP[VLC_AP_idx + RF_AP_num], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, new_serving_UE[VLC_AP_idx + RF_AP_num],
                                   first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);

        // check whether there exists an RU being used multiple times
        existDuplicateRU(new_serving_UE[VLC_AP_idx + RF_AP_num], my_UE_list);
    }


    // step6: consider handover efficiency
    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        int prev_AP = my_UE_list[UE_idx].getPrevAssociatedAP();
        int curr_AP = my_UE_list[UE_idx].getCurrAssociatedAP();

        if (prev_AP > 0) { // VLC
            if (curr_AP > 0 && prev_AP != curr_AP)
                throughput[UE_idx] = throughput[UE_idx] * HHO_efficiency;
            else if (curr_AP == 0)
                throughput[UE_idx] = throughput[UE_idx] * VHO_efficiency;
        }
        else {
            if (curr_AP > 0)
                throughput[UE_idx] = throughput[UE_idx] * HHO_efficiency;
        }

        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }


    // step7: finally, update throughput each UE gets in this state and calculate average satisfaction in each AP
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);
}


/*void partialConfiguration(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                         std::vector<double> &RF_data_rate_vector,
                         std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                         std::vector<std::vector<int>> &AP_association_matrix,
                         std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                         std::vector<int> &rejected_UE,
                         std::vector<double> &discount_ratio_per_AP,
                         std::vector<std::pair<int, int>> &first_empty_RU_position,
                         std::vector<MyUeNode> &my_UE_list)
{
    //step1: construct UE demand vector and serving_UE vector for each AP
    double avg_satisfaction = 0.0;
    std::vector<double> UE_demand (UE_num, 0.0);
    std::vector<std::vector<int>> old_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());
    std::vector<std::vector<int>> new_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int UE_idx = 0; UE_idx < my_UE_list.size(); UE_idx++) {
        avg_satisfaction += my_UE_list[UE_idx].getLastSatisfaction();
        UE_demand[UE_idx] = my_UE_list[UE_idx].getRequiredDataRate();

        if (my_UE_list[UE_idx].getCurrAssociatedAP() >= 0)
            old_serving_UE[my_UE_list[UE_idx].getCurrAssociatedAP()].push_back(UE_idx);
    }
    avg_satisfaction /= UE_num;


    // a little bit difference with the one in completeConfiguration- (-1) -> available, 2 -> this RU is used by UE 2
    std::vector<std::vector<std::vector<int>>> local_RU_matrix (VLC_AP_num, std::vector<std::vector<int>> (effective_subcarrier_num + 1, std::vector<int> (time_slot_num, -1)));
    std::vector<std::pair<int, int>> first_empty_RU_position_copy (VLC_AP_num, std::make_pair(effective_subcarrier_num, time_slot_num - 1));

    // step2: For each VLC AP allocate the least demand to UE who was connected to this AP in the previous state
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        for (int j = 0; j < old_serving_UE[VLC_AP_idx + RF_AP_num].size() && first_empty_RU_position_copy[VLC_AP_idx].first >= 1; j++) {
            int update_flag = 0;
            int UE_idx = old_serving_UE[VLC_AP_idx + RF_AP_num][j];
            int subcarrier_idx = first_empty_RU_position_copy[VLC_AP_idx].first;
            int time_slot_idx = first_empty_RU_position_copy[VLC_AP_idx].second;
            double discounted_demand = UE_demand[UE_idx] * avg_satisfaction; //discount_ratio_per_AP[RF_AP_num + VLC_AP_idx];

            // find the first subcarrier that provides data rate for UE *it
            update_flag = findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

            while (discounted_demand > 0 && subcarrier_idx >= 1) {
                if (local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] == -1) {
                    discounted_demand -= VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                    local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] = UE_idx;
                }
                goToNextRU(subcarrier_idx, time_slot_idx);
            }

            if (update_flag) {
                // find the first available RU
                while (subcarrier_idx >= 1 && local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 1)
                    goToNextRU(subcarrier_idx, time_slot_idx);

                first_empty_RU_position_copy[VLC_AP_idx] = std::make_pair(subcarrier_idx, time_slot_idx);
            }
        }
    }

    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getRequiredDataRate() > b.getRequiredDataRate(); });


    // step3: APA+RA- adjust UEs whose satisfaction is below average
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<double> satisfaction (UE_num, 0.0);

    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();
        int prev_AP = my_UE_list[i].getCurrAssociatedAP(); // since we haven't updated APA results yet, the associated AP in the last state is still recorded in current AP
        double prev_satisfaction = my_UE_list[i].getLastSatisfaction();

        // RUs got in the previous state may not provide the same data rate as the previous one, so it has to be re-estimated
        double prev_throughput = (prev_AP > 0) ? recalculateRuDataRate(VLC_data_rate_matrix[prev_AP - RF_AP_num][UE_idx], my_UE_list[i]) : my_UE_list[i].getLastThroughput();
        //double prev_satisfaction = std::min(prev_throughput / UE_demand[UE_idx], 1.0);


        // satisfaction is above average -> continue to use the current AP and the same RUs, so no need of RA
        if (prev_satisfaction >= avg_satisfaction && prev_throughput >= avg_satisfaction * UE_demand[UE_idx]) {
            my_UE_list[i].setCurrAssociatedAP(prev_AP);
            AP_association_matrix[prev_AP][UE_idx] = 1; // optional
            new_serving_UE[prev_AP].push_back(UE_idx);

            if (prev_AP > 0) {
                throughput[UE_idx] = prev_throughput; // prev_throughput;

                double resource_return = throughput[UE_idx] - avg_satisfaction * UE_demand[UE_idx];
                takeResourceBack(VLC_data_rate_matrix[prev_AP - RF_AP_num][UE_idx], first_empty_RU_position[prev_AP - RF_AP_num], RU_matrix_per_VLC_AP[prev_AP - RF_AP_num],
                                 resource_return, throughput[UE_idx], my_UE_list[i]);

                satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
            }
        }
        else {
            // step3-2-1: find other VLC APs which can provide data rate and which still has available RU
            int chosen_AP = -1;
            double max_offered_data_rate = 0.0;

            for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
                if (VLC_AP_idx != prev_AP - RF_AP_num && VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] > 0.0 && first_empty_RU_position_copy[VLC_AP_idx].first >= 1) {

                    int subcarrier_idx = first_empty_RU_position_copy[VLC_AP_idx].first;
                    int time_slot_idx = first_empty_RU_position_copy[VLC_AP_idx].second;
                    double offered_data_rate = 0.0;
                    double discounted_demand = my_UE_list[i].getRequiredDataRate() * avg_satisfaction; //discount_ratio_per_AP[RF_AP_num + VLC_AP_idx];

                    findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

                    while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
                        if (local_RU_matrix[VLC_AP_idx][subcarrier_idx][time_slot_idx] == -1)
                            offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];

                        goToNextRU(subcarrier_idx, time_slot_idx);
                    }

                    if (offered_data_rate > max_offered_data_rate) {
                        max_offered_data_rate = offered_data_rate;
                        chosen_AP = VLC_AP_idx + RF_AP_num;
                    }
                }
            }

            // if connected to VLC AP in the previous state, then choose the best between RF and VLC
            if (prev_AP > 0) {
                if (RF_data_rate_vector[old_serving_UE[0].size() + 1] > max_offered_data_rate) {
                    chosen_AP = 0;
                    max_offered_data_rate = RF_data_rate_vector[old_serving_UE[0].size() + 1];
                }

                /*double reserved_resource = 0.0;
                for (int subcarrier_idx = 0; subcarrier_idx <= effective_subcarrier_num; subcarrier_idx++) {
                    for (int time_slot_idx = 0; time_slot_idx < time_slot_num; time_slot_idx++) {
                        if (local_RU_matrix[prev_AP - RF_AP_num][subcarrier_idx][time_slot_idx] == UE_idx)
                            reserved_resource += VLC_data_rate_matrix[prev_AP - RF_AP_num][UE_idx][subcarrier_idx];
                    }
                }
                // then compare with the previous AP
                if (reserved_resource > max_offered_data_rate)
                    chosen_AP = prev_AP;*/
            /*}
            if (my_UE_list[i].getLastThroughput() > max_offered_data_rate)
                chosen_AP = prev_AP;


            // step3-2-2: update APA and allocate resource to this UE
            if (chosen_AP == -1)
                rejected_UE.push_back(UE_idx);
            else  {
                AP_association_matrix[chosen_AP][UE_idx] = 1;
                new_serving_UE[chosen_AP].push_back(UE_idx);
            }
            my_UE_list[i].setCurrAssociatedAP(chosen_AP);


            // if this new AP is not the prev AP, then we have to release resource the UE had first
            // and then allocate resource of the new AP to this UE and update the corresponding local RU matrix
            if (chosen_AP != prev_AP) {
                // release resource
                if (prev_AP > 0) {
                    AP_association_matrix[prev_AP][UE_idx] = 0;

                    for (int subcarrier_idx = 0; subcarrier_idx < effective_subcarrier_num + 1; subcarrier_idx++) {
                        for (int time_slot_idx = 0; time_slot_idx < time_slot_num; time_slot_idx++) {
                            if (local_RU_matrix[prev_AP - RF_AP_num][subcarrier_idx][time_slot_idx] == UE_idx)
                                local_RU_matrix[prev_AP - RF_AP_num][subcarrier_idx][time_slot_idx] = -1;
                        }
                    }

                    releaseAllResource(RU_matrix_per_VLC_AP[prev_AP - RF_AP_num], first_empty_RU_position[prev_AP - RF_AP_num], my_UE_list[i]);
                }
                else if (prev_AP == 0) {
                    AP_association_matrix[prev_AP][UE_idx] = 0;

                    std::vector<int>::iterator loc = old_serving_UE[0].begin();
                    for (; loc != old_serving_UE[0].end() && *loc != UE_idx; loc++);

                    if (loc != old_serving_UE[0].end())
                        old_serving_UE[0].erase(loc);
                }

                // allocate resource
                if (chosen_AP > 0) {
                    // update the local RU matrix of the chosen AP
                    int update_flag = 0;
                    int chosen_VLC_AP = chosen_AP - RF_AP_num;
                    int subcarrier_idx = first_empty_RU_position_copy[chosen_VLC_AP].first;
                    int time_slot_idx = first_empty_RU_position_copy[chosen_VLC_AP].second;
                    double offered_data_rate = 0.0;

                    update_flag = findFirstEffectiveSubcarrier(VLC_data_rate_matrix[chosen_VLC_AP][UE_idx], subcarrier_idx, time_slot_idx);

                    while (offered_data_rate < max_offered_data_rate && subcarrier_idx >= 1) {
                        if (local_RU_matrix[chosen_VLC_AP][subcarrier_idx][time_slot_idx] == -1) {
                            offered_data_rate += VLC_data_rate_matrix[chosen_VLC_AP][UE_idx][subcarrier_idx];
                            local_RU_matrix[chosen_VLC_AP][subcarrier_idx][time_slot_idx] = UE_idx;
                        }
                        goToNextRU(subcarrier_idx, time_slot_idx);
                    }

                    if (update_flag) {
                        // find the first available RU
                        while (subcarrier_idx >= 1 && local_RU_matrix[chosen_VLC_AP][subcarrier_idx][time_slot_idx] == 1)
                            goToNextRU(subcarrier_idx, time_slot_idx);

                        first_empty_RU_position_copy[chosen_VLC_AP] = std::make_pair(subcarrier_idx, time_slot_idx);
                    }

                    throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[chosen_VLC_AP][UE_idx], RU_matrix_per_VLC_AP[chosen_VLC_AP], first_empty_RU_position[chosen_VLC_AP],
                                                            max_offered_data_rate, my_UE_list[i]);

                    satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
                }
            }
            else { // chosen_AP == prev_AP
                if (chosen_AP > 0) {
                    throughput[UE_idx] = my_UE_list[i].getLastThroughput();
                    satisfaction[UE_idx] = my_UE_list[i].getLastSatisfaction();
                }
            }
        }
    }

    // UEs served by RF AP are not allocated resource until the entire APA+RA process is done
    for (int i = 0; i < new_serving_UE[0].size(); i++) {
        int UE_idx = new_serving_UE[0][i];

        throughput[UE_idx] = RF_data_rate_vector[new_serving_UE[0].size()];
        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }


    // step3: RRA
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getID() < b.getID(); });

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        residualResourceAllocation(discount_ratio_per_AP[VLC_AP_idx + RF_AP_num], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, new_serving_UE[VLC_AP_idx + RF_AP_num],
                                   first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);

        // check whether there exists an RU being used multiple times
        existDuplicateRU(new_serving_UE[VLC_AP_idx + RF_AP_num], my_UE_list);
    }


    // step4: consider handover efficiency
    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        int prev_AP = my_UE_list[UE_idx].getPrevAssociatedAP();
        int curr_AP = my_UE_list[UE_idx].getCurrAssociatedAP();

        if (prev_AP > 0) { // VLC
            if (curr_AP > 0 && prev_AP != curr_AP)
                throughput[UE_idx] = throughput[UE_idx] * HHO_efficiency;
            else if (curr_AP == 0)
                throughput[UE_idx] = throughput[UE_idx] * VHO_efficiency;
        }
        else {
            if (curr_AP > 0)
                throughput[UE_idx] = throughput[UE_idx] * HHO_efficiency;
        }

        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }


    // step5: finally, update throughput each UE gets in this state and calculate average satisfaction in each AP
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);
}*/

void fullConfiguration(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                         std::vector<double> &RF_data_rate_vector,
                         std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                         std::vector<std::vector<int>> &AP_association_matrix,
                         std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                         std::vector<double> &discount_ratio_per_AP,
                         std::vector<std::pair<int, int>> &first_empty_RU_position,
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


    // step1: record user's demand in another space and sort users in the descending order based on their demands
    std::vector<double> UE_demand;
    for (int i = 0; i < UE_num; i++)
        UE_demand.push_back(my_UE_list[i].getRequiredDataRate());

    // note that be careful of the order of my_UE_list from now on
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getRequiredDataRate() > b.getRequiredDataRate(); });


    // step3: APA + RA
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<double> satisfaction (UE_num, 0.0);
    std::vector<std::vector<int>> serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();

        // <chosen AP, offered data rate> is returned
        std::pair<int, double> result = accessPointAssociation(VLC_data_rate_matrix, RF_data_rate_vector, serving_UE[0], RU_matrix_per_VLC_AP,
                                                                    UE_demand, discount_ratio_per_AP, first_empty_RU_position, my_UE_list[i]);

        int chosen_AP = result.first;
        AP_association_matrix[chosen_AP][UE_idx] = 1;
        my_UE_list[i].setCurrAssociatedAP(chosen_AP);
        serving_UE[chosen_AP].push_back(UE_idx);

        // VLC RA
        if (chosen_AP > 0) {
            throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[chosen_AP - RF_AP_num][UE_idx], RU_matrix_per_VLC_AP[chosen_AP - RF_AP_num],
                                                    first_empty_RU_position[chosen_AP - RF_AP_num], result.second, my_UE_list[i]);

            satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
        }
    }

    // UEs served by RF AP are not allocated resource until the entire APA+RA process is done
    for (int i = 0; i < serving_UE[0].size(); i++) {
        int UE_idx = serving_UE[0][i];

        throughput[UE_idx] = RF_data_rate_vector[serving_UE[0].size()];
        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }


    //step4: RRA
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getID() < b.getID(); });

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        residualResourceAllocation(discount_ratio_per_AP[VLC_AP_idx + RF_AP_num], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, serving_UE[VLC_AP_idx + RF_AP_num],
                                   first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);

        // check whether there exist RU being used multiple times
        existDuplicateRU(serving_UE[VLC_AP_idx + RF_AP_num], my_UE_list);
    }


    // step5: finally, update throughput each UE gets in this state and calculate average satisfaction in each AP
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);
}

void findBestSinrAP(std::vector<int> &best_SINR_AP,
                    std::vector<int> &served_by_RF,
                    std::vector<double> &UE_demand,
                    std::vector<std::vector<int>> &unallocated_UE_under_best_VLC_AP,
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
        if (best_VLC_AP == -1 || best_SINR_vec[1] < 1.0) {
            best_SINR_AP[UE_idx] = 0;
            served_by_RF.push_back(UE_idx);
        }
        else {
            best_SINR_AP[UE_idx] = best_VLC_AP + RF_AP_num;
            unallocated_UE_under_best_VLC_AP[best_VLC_AP].push_back(UE_idx);
        }
    }

    for (int i = 0; i < unallocated_UE_under_best_VLC_AP.size(); i++)
        std::sort(unallocated_UE_under_best_VLC_AP[i].begin(), unallocated_UE_under_best_VLC_AP[i].end(),
                  [&](int UE_a, int UE_b){ return UE_demand[UE_a] > UE_demand[UE_b]; });
}

std::pair<int, double> accessPointAssociation(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                                std::vector<double> &RF_data_rate_vector,
                                                std::vector<int> &served_by_RF,
                                                std::vector<std::vector<std::vector<int>>> RU_matrix_per_VLC_AP,
                                                std::vector<double> &UE_demand,
                                                std::vector<double> &demand_discount_per_AP,
                                                std::vector<std::pair<int, int>> first_empty_RU_position,
                                                MyUeNode &UE_node)
{
    int UE_idx = UE_node.getID();
    std::vector<double> offered_data_rate_per_VLC_AP;

    // step1: for each VLC AP allocate resource to those unallocated UEs first, and then the current UE
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] == 0.0) {
            offered_data_rate_per_VLC_AP.push_back(0.0);
            continue;
        }

        // step1-1: allocate resource to this UE
        int subcarrier_idx = first_empty_RU_position[VLC_AP_idx].first;
        int time_slot_idx = first_empty_RU_position[VLC_AP_idx].second;
        double offered_data_rate = 0.0;
        double discounted_demand = UE_demand[UE_idx] * demand_discount_per_AP[RF_AP_num + VLC_AP_idx];

        // find the first subcarrier that provides data rate for UE *it
        findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

        while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
            if (RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0) {
                offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
                RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] = 1;
            }
            goToNextRU(subcarrier_idx, time_slot_idx);
        }

        offered_data_rate_per_VLC_AP.push_back(offered_data_rate);
    }

    // step2: after all VLC APs finished allocation, choose the one with max offered data rate
    int result_VLC_AP = -1;
    double max_data_rate = 0.0;

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (offered_data_rate_per_VLC_AP[VLC_AP_idx] > max_data_rate) {
            result_VLC_AP = VLC_AP_idx;
            max_data_rate = offered_data_rate_per_VLC_AP[VLC_AP_idx];
        }
    }

    if (max_data_rate > RF_data_rate_vector[served_by_RF.size() + 1])
        return std::make_pair(result_VLC_AP + RF_AP_num, offered_data_rate_per_VLC_AP[result_VLC_AP]);
    else
        return std::make_pair(0, RF_data_rate_vector[served_by_RF.size() + 1]);
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

    if (update_flag) {
        // find the first available RU
        while (subcarrier_idx >= 1 && RU_matrix[subcarrier_idx][time_slot_idx] == 1)
            goToNextRU(subcarrier_idx, time_slot_idx);

        first_empty_RU_position = std::make_pair(subcarrier_idx, time_slot_idx);
    }

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


    // step1: see whether every user's satisfaction is all above discount ratio
    int index = 0;
    for (; index < serving_UE.size(); index++) {
        if (satisfaction[serving_UE[index]] < discount_ratio)
            break;
    }

    // if there exist a user's satisfaction is below discount ratio,
    // then decrease discount ratio, take resource back from those whose satisfaction is higher than discount ratio
    // and allocate returned resource to "below" users
    std::vector<int> below, above;

    if (index != serving_UE.size()) {
        // discount_ratio = std::max(discount_ratio - delta_p, min_discount_ratio);
        discount_ratio -= delta_p;

        for (int i = 0; i < serving_UE.size(); i++) {
            if (satisfaction[serving_UE[i]] > discount_ratio)
                above.push_back(serving_UE[i]);
            else
                below.push_back(serving_UE[i]);
        }

        for (int i = 0; i < above.size(); i++) {
            int target_UE = above[i];
            double resource_return = throughput[target_UE] - discount_ratio * my_UE_list[target_UE].getRequiredDataRate();

            takeResourceBack(VLC_data_rate_matrix[target_UE], first_empty_RU_position, RU_matrix, resource_return, throughput[target_UE], my_UE_list[target_UE]);
        }

        makeUpResourceDifference(discount_ratio, VLC_data_rate_matrix, throughput, satisfaction, below, first_empty_RU_position, RU_matrix, my_UE_list);
    }
    else
        discount_ratio = std::min(discount_ratio + delta_p, 1.0);


    // if still resource not being allocated, then allocate them to users equally
    if (first_empty_RU_position.first >= 1)
        allocateResourceEqually(VLC_data_rate_matrix, throughput, satisfaction, serving_UE, first_empty_RU_position, RU_matrix, my_UE_list);
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

    std::sort(vec.begin(), vec.end(), [](const std::pair<int, double> &a, const std::pair<int, double> &b){ return a.second < b.second; });


    // step2: allocate residual resource to make UEs have minimum discounted data rate
    int index = 0;
    while (first_empty_RU_position.first >= 1 && index < vec.size()) {
        int UE_idx = vec[index].first;
        double diff = vec[index].second;

        throughput[UE_idx] += resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, diff, my_UE_list[UE_idx]);
        index++;
    }


    /*std::sort(target_UE.begin(), target_UE.end(), [&](const int UE_a, const int UE_b){ return satisfaction[UE_a] < satisfaction[UE_b]; });

    int index = 0;
    while (first_empty_RU_position.first >= 1 && index < target_UE.size()) {
        int UE_idx = target_UE[index];
        double diff = (discount_ratio - satisfaction[UE_idx]) * my_UE_list[UE_idx].getRequiredDataRate();

        throughput[UE_idx] += resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, diff, my_UE_list[UE_idx]);
        index++;
    }*/

    updateAllSatisfaction(target_UE, throughput, satisfaction, my_UE_list);
}

void takeResourceBack(std::vector<double> &VLC_data_rate_matrix,
                        std::pair<int, int> &first_empty_RU_position,
                        std::vector<std::vector<int>> &RU_matrix,
                        double resource_return,
                        double &throughput,
                        MyUeNode &UE_node)
{
    if (resource_return <= 0.0)
        return;

    // step1: take back resource allocated to those UE in above vector from low freq to high freq
    int RU_block_idx = UE_node.getRuBlockSize() - 1;

    UE_node.arrangeRuBlock(); // make sure that RU blocks are ordered in the descending frequency order

    while (resource_return > 0 && RU_block_idx >= 0) {
        RuRangeType range = UE_node.getNthResourceUnitBlock(RU_block_idx);
        std::pair<int, int> start = range.second;
        std::pair<int, int> tail = range.first;

        while (resource_return > 0 && (start.first < tail.first || (start.first == tail.first && start.second <= tail.second))) {
            resource_return -= VLC_data_rate_matrix[start.first];
            throughput -= VLC_data_rate_matrix[start.first];

            RU_matrix[start.first][start.second] = 0;
            goToPrevRU(start.first, start.second);
        }

        if (resource_return < 0) {
            goToNextRU(start.first, start.second);
            throughput += VLC_data_rate_matrix[start.first];
            RU_matrix[start.first][start.second] = 1;
        }

        goToNextRU(start.first, start.second); // make "start" point to the last released RU
        if (first_empty_RU_position.first < start.first || (first_empty_RU_position.first == start.first && first_empty_RU_position.second < start.second))
            first_empty_RU_position = start;

        if (start.first == tail.first && start.second == tail.second) {
            UE_node.removeLastResourceUnitBlock();
        }
        else {
            goToPrevRU(start.first, start.second);
            UE_node.updateNthResourceUnitBlock(RU_block_idx, std::make_pair(tail, start));
        }

        RU_block_idx--;
    }
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

    std::sort(serving_UE.begin(), serving_UE.end(), [&](const int UE_a, const int UE_b){ return VLC_data_rate_matrix[UE_a][1] < VLC_data_rate_matrix[UE_b][1]; });

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
        double actual_additional_gained = resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, estimated_additional_gained, my_UE_list[UE_idx]);

        throughput[UE_idx] += actual_additional_gained;

        // resource that the next user would have is the average of total_additional_resource over the rest of users
        total_additional_resource -= actual_additional_gained;

        if (i != serving_UE.size()-1)
            estimated_additional_gained = total_additional_resource / (serving_UE.size()-i-1);
    }

    updateAllSatisfaction(serving_UE, throughput, satisfaction, my_UE_list);
}

void updateOneSatisfaction(double throughput, double &satisfaction, MyUeNode &UE_node)
{
    satisfaction = std::min(throughput / UE_node.getRequiredDataRate(), 1.0);
}

void updateAllSatisfaction(std::vector<int> &serving_UE, std::vector<double> &throughput, std::vector<double> &satisfaction, std::vector<MyUeNode> &my_UE_list)
{
    for (int i = 0; i < serving_UE.size(); i++) {
        int UE_idx = serving_UE[i];

        updateOneSatisfaction(throughput[UE_idx], satisfaction[UE_idx], my_UE_list[UE_idx]);
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

void releaseAllResource(std::vector<std::vector<int>> &RU_matrix, std::pair<int, int> &first_empty_RU_position, MyUeNode &UE_node)
{
    std::vector<RuRangeType> RU_block = UE_node.getWholeRuBlock();

    for (int i = 0; i < RU_block.size(); i++) {
        std::pair<int, int> head = RU_block[i].first;
        std::pair<int, int> tail = RU_block[i].second;

        while (head.first > tail.first || (head.first == tail.first && head.second >= tail.second)) {
            RU_matrix[head.first][head.second] = 0;

            if (first_empty_RU_position.first < head.first || (first_empty_RU_position.first == head.first && first_empty_RU_position.second < head.second))
                first_empty_RU_position = head;

            goToNextRU(head.first, head.second);
        }
    }

    UE_node.clearRuBlock();
}

void existDuplicateRU(std::vector<int> &serving_UE, std::vector<MyUeNode> &my_UE_list)
{
    std::vector<std::vector<int>> duplicate(effective_subcarrier_num + 1, std::vector<int> (time_slot_num, 0));

    for (int i = 0; i < serving_UE.size(); i++) {
        int UE_idx = serving_UE[i];

        for (int j = 0; j < my_UE_list[UE_idx].getRuBlockSize(); j++) {
            RuRangeType block = my_UE_list[UE_idx].getNthResourceUnitBlock(j);
            std::pair<int, int> head = block.first;
            std::pair<int, int> tail = block.second;

            while (head.first > tail.first || (head.first == tail.first && head.second >= tail.second)) {
                if (duplicate[head.first][head.second] > 0) {
                    std::cout << "Error: RU is used multiple times\n";
                    exit(0);
                }

                duplicate[head.first][head.second] += 1;
                goToNextRU(head.first, head.second);
            }
        }
    }
}

double recalculateRuDataRate(std::vector<double> &VLC_data_rate_matrix, MyUeNode &UE_node)
{
    double new_data_rate = 0.0;

    for (int i = 0; i < UE_node.getRuBlockSize(); i++) {
        RuRangeType RU_range = UE_node.getNthResourceUnitBlock(i);
        std::pair<int, int> start = RU_range.first;
        std::pair<int, int> tail = RU_range.second;

        while (start.first > tail.first || (start.first == tail.first && start.second >= tail.second)) {
            new_data_rate += VLC_data_rate_matrix[start.first];

            goToNextRU(start.first, start.second);
        }
    }

    return new_data_rate;
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

void updateInfoToMyUeList(std::vector<double> &throughput,
                          std::vector<double> &satisfaction,
                          std::vector<MyUeNode> &my_UE_list)
{
    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        my_UE_list[UE_idx].addThroughput(throughput[UE_idx]);
        my_UE_list[UE_idx].addSatisfaction(satisfaction[UE_idx]);
    }
}
