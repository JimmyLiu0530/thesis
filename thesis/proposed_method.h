#ifndef PROPOSED_METHOD_H
#define PROPOSED_METHOD_H


#include "my_UE_node.h"
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
                           std::vector<MyUeNode> &my_UE_list);


void proposedMethodForState0(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<MyUeNode> &my_UE_list);

void proposedMethodForStateN(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<MyUeNode> &my_UE_list);

std::vector<int> sortApBasedOnResidualResource(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                               std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                                               MyUeNode &UE);

bool checkSatisfactionUnderAP(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                              std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                              std::unordered_set<int> &unallocated_UE_set,
                              std::vector<MyUeNode> my_UE_list,
                              int VLC_AP_index,
                              int UE_index);

#endif // PROPOSED_METHOD_H
