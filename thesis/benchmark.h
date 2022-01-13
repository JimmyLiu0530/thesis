#ifndef BENCHMARK_H
#define BENCHMARK_H


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"
#include "global_environment.h"


void benchmarkDynamicLB(int state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                       std::vector<std::vector<double>> &AP_sssociation_matrix,
                       std::vector<MyUeNode> &my_UE_list);

void algorithmForFirstState(std::vector<double> &RF_data_rate_vector,
                            std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                            std::vector<std::vector<double>> &AP_sssociation_matrix,
                            std::vector<MyUeNode> &my_UE_list);

void algorithmExceptFirstState();


void updateApAssociationResult(std::vector<MyUeNode> &my_UE_list,
                               std::vector<int> &serving_AP,
                               std::vector<std::vector<int>> &AP_sssociation_matrix);


#endif // BENCHMARK_H
