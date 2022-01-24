#ifndef BENCHMARK_H
#define BENCHMARK_H


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"
#include "global_configuration.h"


void benchmarkDynamicLB(int &state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                       std::vector<std::vector<int>> &AP_association_matrix,
                       std::vector<MyUeNode> &my_UE_list);

std::vector<int> initializeStep(std::vector<double> &RF_data_rate_vector,
                                std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                std::vector<std::vector<int>> &AP_association_matrix,
                                std::vector<MyUeNode> &my_UE_list);

std::vector<int> EGT_basedLoadBalance(std::vector<double> &RF_data_rate_vector,
                                        std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                        std::vector<std::vector<int>> &AP_association_matrix,
                                        std::vector<MyUeNode> &my_UE_list);


std::vector<double> OFDMA(int VLC_AP_index, std::vector<int> served_UE, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix);

void updateApAssociationResult(std::vector<MyUeNode> &my_UE_list,
                               std::vector<int> &serving_AP,
                               std::vector<std::vector<int>> &AP_association_matrix);

void updateResourceAllocation(std::vector<MyUeNode> &my_UE_list, std::vector<double> &throughput);

std::vector<int> constructServedUeSet(std::vector<std::vector<int>> &AP_association_matrix, int AP_index);

#endif // BENCHMARK_H
