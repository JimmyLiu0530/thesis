#ifndef BENCHMARK_H
#define BENCHMARK_H


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"
#include "global_configuration.h"


double benchmarkDynamicLB(int &state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                       std::vector<std::vector<int>> &AP_association_matrix,
                       std::vector<MyUeNode> &my_UE_list);

std::vector<int> initializedStep(std::vector<double> &RF_data_rate_vector,
                                std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                std::vector<std::vector<int>> &local_AP_association_matrix,
                                std::vector<std::vector<double>> &throughtput_per_iteration);

std::vector<int> EGT_basedLoadBalance(std::vector<double> &RF_data_rate_vector,
                                        std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                        std::vector<std::vector<int>> &local_AP_association_matrix,
                                        std::vector<std::vector<double>> &throughtput_per_iteration);


std::vector<double> OFDMA(int VLC_AP_index, std::vector<int> &served_UE, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix);


std::vector<int> constructServedUeSet(std::vector<std::vector<int>> &local_AP_association_matrix, int AP_index);


void updateApAssociationResult(std::vector<std::vector<int>> &local_AP_sssociation_matrix,
                               std::vector<std::vector<int>> &AP_sssociation_matrix,
                               std::vector<MyUeNode> &my_UE_list);

void updateResourceAllocationResult(std::vector<std::vector<double>> &throughtput_per_iteration, std::vector<MyUeNode> &my_UE_list);

#endif // BENCHMARK_H
