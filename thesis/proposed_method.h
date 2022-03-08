#ifndef PROPOSED_METHOD_H
#define PROPOSED_METHOD_H

#include<tuple>

#include "my_UE_node.h"
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
                       std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                       std::vector<double> &demand_discount_per_AP,
                       std::vector<std::tuple<int, int>> &first_empty_RU_position,
                       std::vector<MyUeNode> &my_UE_list);


void proposedMethodForState0(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                             std::vector<int> &rejected_UE,
                             std::vector<double> &demand_discount_per_AP,
                             std::vector<std::tuple<int, int>> &first_empty_RU_position,
                             std::vector<MyUeNode> &my_UE_list);

void proposedMethodForStateN(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                             std::vector<int> &rejected_UE,
                             std::vector<double> &demand_discount_per_AP,
                             std::vector<std::tuple<int, int>> &first_empty_RU_position,
                             std::vector<MyUeNode> &my_UE_list);

void findBestSinrAP(std::vector<int> &best_SINR_AP,
                    std::vector<std::unordered_set<int>> &unallocated_UE_under_best_VLC_AP,
                    std::vector<int> &served_by_RF,
                    std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix);

std::tuple<int, double> accessPointAssociation(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                                std::vector<double> &RF_data_rate_vector,
                                                std::vector<int> &served_by_RF;
                                                std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                                                std::vector<std::unordered_set<int>> &unallocated_UE_under_best_VLC_AP,
                                                std::vector<double>& UE_demand,
                                                std::vector<double> &demand_discount_per_AP,
                                                std::vector<std::tuple<int, int>> &first_empty_RU_position,
                                                MyUeNode &UE_node);

double resourceAllocation(std::vector<std::vector<double>> &VLC_data_rate_matrix,
                            std::vector<std::vector<int>> &RU_matrix,
                            std::tuple<int, int> &first_empty_RU_position,
                            double offered_data_rate,
                            MyUeNode &UE_node);

std::vector<int> sortApBasedOnResidualResource(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                               std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                                               MyUeNode &UE);

bool checkSatisfactionUnderAP(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                              std::vector<std::vector<std::vector<int>>> &RU_matrix_per_VLC_AP,
                              std::unordered_set<int> &unallocated_UE_set,
                              std::vector<double> &UE_demand,
                              int VLC_AP_index,
                              int UE_index);


void findFirstEffectiveSubcarrier(std::vector<double> &VLC_data_rate_matrix, int &subcarrier_index, int &time_slot_idx, int &update_flag);
void backToLastRU(int &subcarrier_idx, int &time_slot_idx);
void goToNextRU(int &subcarrier_idx, int &time_slot_idx);
#endif // PROPOSED_METHOD_H
