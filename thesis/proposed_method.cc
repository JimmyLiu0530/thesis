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
#include <string>

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

#if HIGH_PERFORMANCE
    if (state % complete_config_period == 0)
        lowComplexity(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                          discount_ratio_per_AP, first_empty_RU_position, my_UE_list);
    else
        highPerformance(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP, first_empty_RU_position, my_UE_list);

#else
    lowComplexity(VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                    discount_ratio_per_AP, first_empty_RU_position, my_UE_list);
#endif // HIGH_PERFORMANCE


#if DEBUG_MODE

    for (int i = 0; i < VLC_AP_num; i++) {
        printResourceUnitMatrix(RU_matrix_per_VLC_AP, i);
    }

    for (int i = 0; i < UE_num; i++) {
        int counter = 0;
        for (int j = 0; j < VLC_AP_num+RF_AP_num; j++) {
            if (AP_association_matrix[j][i] == 1) {
                if (counter > 1) {
                    std::cout << "AP association matrix is wrong\n";
                    exit(EXIT_FAILURE);
                }
                else
                    counter++;
            }
        }
    }

#endif // DEBUG_MODE
}

void highPerformance(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                         std::vector<double> &RF_data_rate_vector,
                         std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                         std::vector<std::vector<int>> &AP_association_matrix,
                         std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                         std::vector<std::pair<int, int>> &first_empty_RU_position,
                         std::vector<MyUeNode> &my_UE_list)
{
    //step1: calculate average satisfaction of the previous state.
    double avg_satisfaction = 0.0;
    const std::vector<double> UE_demand = createDemandVector(my_UE_list);
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<double> satisfaction (UE_num, 0.0);
    std::vector<double> avg_satisfaction_per_VLC_AP (VLC_AP_num, 0.0);
    std::vector<std::vector<int>> old_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());
    std::vector<std::vector<int>> new_serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());


    for (int UE_idx = 0; UE_idx < my_UE_list.size(); UE_idx++) {
        int prev_AP = my_UE_list[UE_idx].getCurrAssociatedAP();

        old_serving_UE[prev_AP].push_back(UE_idx);
        avg_satisfaction += my_UE_list[UE_idx].getLastSatisfaction();

        if (prev_AP > 0) {
            throughput[UE_idx] = recalculateRuDataRate(VLC_data_rate_matrix[prev_AP - RF_AP_num][UE_idx], RU_matrix_per_VLC_AP[prev_AP - RF_AP_num],
                                                       first_empty_RU_position[prev_AP - RF_AP_num], my_UE_list[UE_idx]);
        }
        else {
            throughput[UE_idx] = my_UE_list[UE_idx].getLastThroughput();
        }

        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }
    avg_satisfaction /= UE_num;


    // step2: replace low-freq RUs with high-freq RUs for each VLC AP
    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (old_serving_UE[VLC_AP_idx + RF_AP_num].empty())
            continue;

        int AP_idx = VLC_AP_idx + RF_AP_num;
        std::sort(old_serving_UE[AP_idx].begin(), old_serving_UE[AP_idx].end(),
                  [&](int UE_a, int UE_b){ return VLC_SINR_matrix[VLC_AP_idx][UE_a][1] > VLC_SINR_matrix[VLC_AP_idx][UE_b][1]; });

        for (int i = 0; i < old_serving_UE[AP_idx].size(); i++) {
            int UE_idx = old_serving_UE[AP_idx][i];

            releaseAllResource(RU_matrix_per_VLC_AP[VLC_AP_idx], first_empty_RU_position[VLC_AP_idx], my_UE_list[UE_idx]);
            throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], first_empty_RU_position[VLC_AP_idx],
                                                        throughput[UE_idx], my_UE_list[UE_idx]);

            satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
            avg_satisfaction_per_VLC_AP[VLC_AP_idx] += satisfaction[UE_idx];
        }

        avg_satisfaction_per_VLC_AP[VLC_AP_idx] /= old_serving_UE[AP_idx].size();
    }

    // step3: divide APs into two groups
    std::vector<int> higher_than_avg;
    std::vector<int> lower_than_avg;

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (avg_satisfaction_per_VLC_AP[VLC_AP_idx] > avg_satisfaction)
            higher_than_avg.push_back(VLC_AP_idx + RF_AP_num);

        else
            lower_than_avg.push_back(VLC_AP_idx + RF_AP_num);
    }

    // step3-1: for each AP in the "higher" group
    for (int i = 0; i < higher_than_avg.size(); i++) {
        int AP_idx = higher_than_avg[i];
        std::vector<int> beyond, below;

        for (int j = 0; j < old_serving_UE[AP_idx].size(); j++) {
            int UE_idx = old_serving_UE[AP_idx][j];

            if (satisfaction[UE_idx] > avg_satisfaction)
                beyond.push_back(UE_idx);
            else if (satisfaction[UE_idx] < avg_satisfaction)
                below.push_back(UE_idx);
        }

        // beyond group
        for (int j = 0; j < beyond.size(); j++) {
            int UE_idx = beyond[j];
            int VLC_AP_idx = AP_idx - RF_AP_num;

            takeResourceBack(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx],
                             throughput[UE_idx] - avg_satisfaction * UE_demand[UE_idx], throughput[UE_idx], my_UE_list[UE_idx]);

            satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
        }

        // below group
        makeUpResourceDifference(avg_satisfaction, VLC_data_rate_matrix[AP_idx - RF_AP_num], throughput, satisfaction, below, first_empty_RU_position[AP_idx - RF_AP_num],
                                 RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], my_UE_list);

    }

    // step3-2: for each AP in the "lower" group
    for (int i = 0; i < lower_than_avg.size(); i++) {

        int AP_idx = lower_than_avg[i];
        std::vector<int> beyond, below;

        for (int j = 0; j < old_serving_UE[AP_idx].size(); j++) {
            int UE_idx = old_serving_UE[AP_idx][j];

            if (satisfaction[UE_idx] > avg_satisfaction)
                beyond.push_back(UE_idx);
            else if (satisfaction[UE_idx] < avg_satisfaction)
                below.push_back(UE_idx);
        }

        // beyond group
        for (int j = 0; j < beyond.size(); j++) {
            int UE_idx = beyond[j];
            int VLC_AP_idx = AP_idx - RF_AP_num;

            takeResourceBack(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx],
                             throughput[UE_idx] - avg_satisfaction * UE_demand[UE_idx], throughput[UE_idx], my_UE_list[UE_idx]);

            satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
        }


        makeUpResourceDifference(avg_satisfaction, VLC_data_rate_matrix[AP_idx - RF_AP_num], throughput, satisfaction, below, first_empty_RU_position[AP_idx - RF_AP_num],
                                 RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], my_UE_list);

        std::vector<int>::iterator it = below.begin();
        while (it != below.end()) {
            int UE_idx = *it;

            if (satisfaction[UE_idx] < avg_satisfaction_per_VLC_AP[AP_idx - RF_AP_num]) {
                throughput[UE_idx] = 0.0;
                satisfaction[UE_idx] = 0.0;
                releaseAllResource(RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], first_empty_RU_position[AP_idx - RF_AP_num], my_UE_list[UE_idx]);

                it = below.erase(it);
            }
            else
                it++;
        }

        makeUpResourceDifference(avg_satisfaction, VLC_data_rate_matrix[AP_idx - RF_AP_num], throughput, satisfaction, below, first_empty_RU_position[AP_idx - RF_AP_num],
                                 RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], my_UE_list);


        /*std::sort(below.begin(), below.end(), [&](int UE_a, int UE_b){ return (avg_satisfaction * UE_demand[UE_a] - throughput[UE_a]) >
                                                                            (avg_satisfaction * UE_demand[UE_b] - throughput[UE_b]); });

        int expel_num = (int) std::floor(expel_ratio * below.size());
        for (int j = 0; j < expel_num; j++) {
            int UE_idx = below[j];

            throughput[UE_idx] = 0.0;
            satisfaction[UE_idx] = 0.0;
            releaseAllResource(RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], first_empty_RU_position[AP_idx - RF_AP_num], my_UE_list[UE_idx]);
        }

        std::vector<int> new_below (below.begin() + expel_num, below.end());
        makeUpResourceDifference(avg_satisfaction, VLC_data_rate_matrix[AP_idx - RF_AP_num], throughput, satisfaction, new_below, first_empty_RU_position[AP_idx - RF_AP_num],
                                 RU_matrix_per_VLC_AP[AP_idx - RF_AP_num], my_UE_list);*/
    }


    // step4: sort UEs in the descending order of demand
    std::sort(my_UE_list.begin(), my_UE_list.end(), [&](MyUeNode a, MyUeNode b){ return satisfaction[a.getID()] < satisfaction[b.getID()]; });

    // step5: APA+RA
    for (int i = 0; i < my_UE_list.size(); i++) {
        int UE_idx = my_UE_list[i].getID();
        int prev_AP = my_UE_list[i].getCurrAssociatedAP(); // since we haven't updated APA results yet, the associated AP in the last state is still recorded in current AP

        // satisfaction is above average -> continue to use the current AP and the same RUs, so no need of RA
        if (prev_AP > 0 && satisfaction[UE_idx] >= avg_satisfaction) {
            my_UE_list[i].setCurrAssociatedAP(prev_AP);
            AP_association_matrix[prev_AP][UE_idx] = 1; // optional
            new_serving_UE[prev_AP].push_back(UE_idx);
        }
        else {
            // step4-2-1: find other VLC APs which can provide data rate and which still has available RU
            std::vector<std::pair<int, double>> candidate_VLC_AP;

            for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
                if (VLC_AP_idx != prev_AP - RF_AP_num && VLC_data_rate_matrix[VLC_AP_idx][UE_idx][1] > 0.0 && first_empty_RU_position[VLC_AP_idx].first >= 1) {
                    int subcarrier_idx = first_empty_RU_position[VLC_AP_idx].first;
                    int time_slot_idx = first_empty_RU_position[VLC_AP_idx].second;
                    double offered_data_rate = 0.0;
                    double discounted_demand = (prev_AP == 0 && satisfaction[UE_idx] >= avg_satisfaction) ? UE_demand[UE_idx] : avg_satisfaction * UE_demand[UE_idx];

                    findFirstEffectiveSubcarrier(VLC_data_rate_matrix[VLC_AP_idx][UE_idx], subcarrier_idx, time_slot_idx);

#if DEBUG_MODE
                    printResourceUnitMatrix(RU_matrix_per_VLC_AP, VLC_AP_idx);
#endif // DEGUB_MODE

                    while (offered_data_rate < discounted_demand && subcarrier_idx >= 1) {
                        if (RU_matrix_per_VLC_AP[VLC_AP_idx][subcarrier_idx][time_slot_idx] == 0)
                            offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];

                        goToNextRU(subcarrier_idx, time_slot_idx);
                    }

                    if (offered_data_rate > throughput[UE_idx] || offered_data_rate >= discounted_demand)
                        candidate_VLC_AP.emplace_back(VLC_AP_idx + RF_AP_num, offered_data_rate);
                }
            }

            if (prev_AP > 0) {
                if (RF_data_rate_vector[old_serving_UE[0].size() + 1] > throughput[UE_idx])
                    candidate_VLC_AP.emplace_back(0, RF_data_rate_vector[old_serving_UE[0].size() + 1]);
            }

            // if the candidate set is not empty, choose the AP with the most data rate from it.
            int new_AP = prev_AP;
            double max_offered_data_rate = 0.0;

            if (!candidate_VLC_AP.empty()) {
                int best_index = 0;
                for (int i = 1; i < candidate_VLC_AP.size(); i++) {
                    best_index = (candidate_VLC_AP[i].second > candidate_VLC_AP[best_index].second) ? i : best_index;
                }

                new_AP = candidate_VLC_AP[best_index].first;
                max_offered_data_rate = candidate_VLC_AP[best_index].second;
            }

            // step4-2-2: update APA and allocate resource to this UE
            new_serving_UE[new_AP].push_back(UE_idx);
            my_UE_list[i].setCurrAssociatedAP(new_AP);
            AP_association_matrix[new_AP][UE_idx] = 1;

            if (new_AP != prev_AP) {
                // release resource
                AP_association_matrix[prev_AP][UE_idx] = 0;

                if (prev_AP > 0) {
                    releaseAllResource(RU_matrix_per_VLC_AP[prev_AP - RF_AP_num], first_empty_RU_position[prev_AP - RF_AP_num], my_UE_list[i]);
                }
                else if (prev_AP == 0) {
                    int new_UE_num = old_serving_UE[0].size() - 1;

                    for (int j = 0; j < old_serving_UE[0].size(); j++) {
                        int RF_UE_idx = old_serving_UE[0][j];

                        if (RF_UE_idx == UE_idx) {
                            old_serving_UE[0].erase(old_serving_UE[0].begin() + j);
                            j--;
                        }
                        else {
                            throughput[RF_UE_idx] = RF_data_rate_vector[new_UE_num];
                            satisfaction[RF_UE_idx] = std::min(throughput[RF_UE_idx] / UE_demand[RF_UE_idx], 1.0);
                        }
                    }
                }

                // allocate resource
                if (new_AP > 0) {
                    int new_VLC_AP = new_AP - RF_AP_num;
                    throughput[UE_idx] = resourceAllocation(VLC_data_rate_matrix[new_VLC_AP][UE_idx], RU_matrix_per_VLC_AP[new_VLC_AP], first_empty_RU_position[new_VLC_AP],
                                                            max_offered_data_rate, my_UE_list[i]);

                    satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
                }
                else if (new_AP == 0)
                    old_serving_UE[0].push_back(UE_idx);
            }
        }
    }

    // UEs served by RF AP are not allocated resource until the entire APA+RA process is done
    for (int i = 0; i < new_serving_UE[0].size(); i++) {
        int UE_idx = new_serving_UE[0][i];

        throughput[UE_idx] = RF_data_rate_vector[new_serving_UE[0].size()];
        satisfaction[UE_idx] = std::min(throughput[UE_idx] / UE_demand[UE_idx], 1.0);
    }

    // step6: RRA
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getID() < b.getID(); });

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        if (new_serving_UE[VLC_AP_idx + RF_AP_num].empty())
            continue;

        int next_flag = 0;
        int AP_idx = VLC_AP_idx + RF_AP_num;
        std::vector<int> target_UE;
        std::map<int, double> old_UE_throughput_mapping;

        for (int i = 0; i < new_serving_UE[AP_idx].size(); i++) {
            target_UE.push_back(new_serving_UE[AP_idx][i]);
            old_UE_throughput_mapping[new_serving_UE[AP_idx][i]] = throughput[new_serving_UE[AP_idx][i]];
        }

        do {
            next_flag = 0;

            // find the best satisfaction
            double best_satis = 0.0;
            for (int i = 0; i < target_UE.size(); i++) {
                best_satis = std::max(best_satis, satisfaction[target_UE[i]]);
            }

            makeUpResourceDifference(std::min(best_satis + delta_p, 1.0), VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, target_UE, first_empty_RU_position[VLC_AP_idx],
                                     RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);

            // check if next iteration is needed
            std::map<int, double>::iterator it = old_UE_throughput_mapping.begin();
            while (it != old_UE_throughput_mapping.end()) {
                if (it->second != throughput[it->first]) {
                    next_flag = 1;
                    it->second = throughput[it->first];
                    it++;
                }
                else {
                    int loc = 0;
                    for (; loc < target_UE.size() && target_UE[loc] != it->first; loc++);

                    if (loc < target_UE.size())
                        target_UE.erase(target_UE.begin() + loc);

                    it = old_UE_throughput_mapping.erase(it);
                }
            }
        } while (next_flag);

        int best_SINR_UE = new_serving_UE[AP_idx][0];
        for (int i = 1; i < new_serving_UE[AP_idx].size(); i++) {
            if (VLC_SINR_matrix[VLC_AP_idx][new_serving_UE[AP_idx][i]][1] > VLC_SINR_matrix[VLC_AP_idx][best_SINR_UE][1])
                best_SINR_UE = new_serving_UE[AP_idx][i];
        }

        throughput[best_SINR_UE] += resourceAllocation(VLC_data_rate_matrix[VLC_AP_idx][best_SINR_UE], RU_matrix_per_VLC_AP[VLC_AP_idx], first_empty_RU_position[VLC_AP_idx],
                                                        std::numeric_limits<double>::max(), my_UE_list[best_SINR_UE]);

        satisfaction[best_SINR_UE] = std::min(throughput[best_SINR_UE] / UE_demand[best_SINR_UE], 1.0);

 #if DEBUG_MODE
        existDuplicateRU(new_serving_UE[VLC_AP_idx + RF_AP_num], my_UE_list);
#endif // DEBUG_MODE
    }

    // finally, update throughput each UE gets in this state and calculate average satisfaction in each AP
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);
}

void lowComplexity(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
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
    const std::vector<double> UE_demand = createDemandVector(my_UE_list);


    // step2: sort UEs in the descending order of demand
    // note that be careful of the order of my_UE_list from now on
    if (my_UE_list[0].getSatisfactionHistory().empty())
        std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getRequiredDataRate() > b.getRequiredDataRate(); });
    else
        std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){ return a.getLastSatisfaction() < b.getLastSatisfaction(); });


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
        residualResourceAllocation(discount_ratio_per_AP[VLC_AP_idx + RF_AP_num], VLC_SINR_matrix[VLC_AP_idx], VLC_data_rate_matrix[VLC_AP_idx], throughput, satisfaction, serving_UE[VLC_AP_idx + RF_AP_num],
                                   first_empty_RU_position[VLC_AP_idx], RU_matrix_per_VLC_AP[VLC_AP_idx], my_UE_list);

#if DEBUG_MODE
        existDuplicateRU(serving_UE[VLC_AP_idx + RF_AP_num], my_UE_list);
#endif // DEBUG_MODE
    }

    // step5: finally, update throughput each UE gets in this state and calculate average satisfaction in each AP
    updateInfoToMyUeList(throughput, satisfaction, my_UE_list);
}

std::pair<int, double> accessPointAssociation(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                                std::vector<double> &RF_data_rate_vector,
                                                std::vector<int> &served_by_RF,
                                                std::vector<std::vector<std::vector<int>>> RU_matrix_per_VLC_AP,
                                                const std::vector<double> UE_demand,
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

    // modification: add the second condition to avoid too many users connected to RF
    if (max_data_rate > RF_data_rate_vector[served_by_RF.size() + 1] || max_data_rate >= UE_demand[UE_idx] * demand_discount_per_AP[result_VLC_AP + RF_AP_num])
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

#if DEBUG_MODE
        printResourceUnitMatrix(RU_matrix);
#endif // DEBUG_MODE

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
                                std::vector<std::vector<double>> &VLC_SINR_matrix,
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

    // step2: see whether every user's satisfaction is all above discount ratio
    int index = 0;
    for (; index < serving_UE.size(); index++) {
        if (satisfaction[serving_UE[index]] < discount_ratio)
            break;
    }

    std::vector<int> above, below;
    if (index != serving_UE.size()) {
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
            satisfaction[target_UE] = std::min(throughput[target_UE] / my_UE_list[target_UE].getRequiredDataRate(), 1.0);
        }

        makeUpResourceDifference(discount_ratio, VLC_data_rate_matrix, throughput, satisfaction, below, first_empty_RU_position, RU_matrix, my_UE_list);
    }
    else
        discount_ratio = std::min(discount_ratio + delta_p, 1.0);


    allocateResourceEqually(VLC_SINR_matrix, VLC_data_rate_matrix, throughput, satisfaction, serving_UE, first_empty_RU_position, RU_matrix, my_UE_list);
}


// allocate residual resource to users in the manner that a UE reaching the discount_ratio with the least data rate is allocated first.
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

    // std::sort(target_UE.begin(), target_UE.end(), [&](int UE_a, int UE_b){ return satisfaction[UE_a] < satisfaction[UE_b]; });
    std::sort(target_UE.begin(), target_UE.end(), [&](int UE_a, int UE_b){ return (discount_ratio * my_UE_list[UE_a].getRequiredDataRate() - throughput[UE_a]) >
                                                                            (discount_ratio * my_UE_list[UE_b].getRequiredDataRate() - throughput[UE_b]); });

    int index = 0;
    while (first_empty_RU_position.first >= 1 && index < target_UE.size()) {
        int UE_idx = target_UE[index];
        double diff = my_UE_list[UE_idx].getRequiredDataRate() * discount_ratio - throughput[UE_idx];

        throughput[UE_idx] += resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, diff, my_UE_list[UE_idx]);
        satisfaction[UE_idx] = std::min(throughput[UE_idx] / my_UE_list[UE_idx].getRequiredDataRate(), 1.0);
        index++;
    }
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

    UE_node.arrangeRuBlock(high_to_low); // make sure that RU blocks are ordered in the descending order of frequency

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

        // make sure that the amount of released resource does not exceed "resource_return"
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
void allocateResourceEqually(std::vector<std::vector<double>> &VLC_SINR_matrix,
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

    int best_UE = serving_UE[0];
    for (int i = 1; i < serving_UE.size(); i++) {
        if (VLC_SINR_matrix[serving_UE[i]][1] > VLC_SINR_matrix[best_UE][1])
            best_UE = serving_UE[i];
    }

    double total_resource = 0.0;
    for (int i = 1; i < effective_subcarrier_num + 1; i++) {
        for (int j = 0; j < time_slot_num; j++) {
            total_resource += (1 - RU_matrix[i][j]) * VLC_data_rate_matrix[best_UE][i];
        }
    }

    double estimated_additional_gained = total_resource / serving_UE.size();


    // if AP still has residual resource, allocate equally
    if (total_resource > 0.0) {
        std::sort(serving_UE.begin(), serving_UE.end(), [&](const int UE_a, const int UE_b){ return VLC_SINR_matrix[UE_a][1] < VLC_SINR_matrix[UE_b][1]; });

        estimated_additional_gained = total_resource / serving_UE.size();
        for (int i = 0; i < serving_UE.size(); i++) {
            int UE_idx = serving_UE[i];
            double actual_additional_gained = resourceAllocation(VLC_data_rate_matrix[UE_idx], RU_matrix, first_empty_RU_position, estimated_additional_gained, my_UE_list[UE_idx]);

            // resource that the next user would have is the average of total_resource over the rest of users
            total_resource -= actual_additional_gained;
            throughput[UE_idx] += actual_additional_gained;
            satisfaction[UE_idx] = std::min(throughput[UE_idx] / my_UE_list[UE_idx].getRequiredDataRate(), 1.0);

            if (i != serving_UE.size() - 1)
                estimated_additional_gained = total_resource / (serving_UE.size() - i - 1);
        }
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

/* functionality:
 * 1. recalculate how much data rate the RUs the UE has provide
 * 2. release RUs which cannot provide any data rate for the UE
 */
double recalculateRuDataRate(std::vector<double> &VLC_data_rate_matrix, std::vector<std::vector<int>> &RU_matrix, std::pair<int, int> &first_empty_RU_position, MyUeNode &UE_node)
{
    // int replace_flag = 1;
    double new_data_rate = 0.0;

    for (int i = 0; i < UE_node.getRuBlockSize(); i++) {
        RuRangeType RU_range = UE_node.getNthResourceUnitBlock(i);
        std::pair<int, int> start = RU_range.first;
        std::pair<int, int> tail = RU_range.second;

        while ((start.first > tail.first || (start.first == tail.first && start.second >= tail.second)) && VLC_data_rate_matrix[start.first] == 0.0) {
            // replace_flag = 0;
            RU_matrix[start.first][start.second] = 0;
            goToNextRU(start.first, start.second);
        }

        if (start.first < tail.first || (start.first == tail.first && start.second < tail.second)) {
            UE_node.removeNthResourceUnitBlock(i);
            i--;
        }
        else {
            UE_node.updateNthResourceUnitBlock(i, std::make_pair(start, tail));

            while (start.first > tail.first || (start.first == tail.first && start.second >= tail.second)) {
                new_data_rate += VLC_data_rate_matrix[start.first];
                goToNextRU(start.first, start.second);
            }
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

std::vector<double> createDemandVector(std::vector<MyUeNode> &my_UE_list) {
    std::vector<double> demands;

    for (int i = 0; i < my_UE_list.size(); i++)
        demands.push_back(my_UE_list[i].getRequiredDataRate());

    return demands;
}
