#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <random>
#include <chrono>


#include "benchmark.h"
#include "channel.h"
#include "my_UE_node.h"
#include "print.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"




void benchmarkDynamicLB(int state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                       std::vector<std::vector<double>> &AP_sssociation_matrix,
                       std::vector<MyUeNode> &my_UE_list)
{
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);

    if (state == 0)
        algorithmForFirstState();
    else
        algorithmExceptFirstState();


    state++;
}


void algorithmForFirstState(std::vector<double> &RF_data_rate_vector,
                            std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                            std::vector<std::vector<double>> &AP_sssociation_matrix,
                            std::vector<MyUeNode> &my_UE_list)
{
    // step1: construct strategy sets of the form {RF_AP, best VLC_AP} for all UE
    std::vector<std::vector<int>> strategy_set (UE_num, std::vector<int> (2, -1));

    for (int i = 0; i < UE_num; i++) {
        strategy_set[i][0] = 0; // strategy set always contains the RF AP

        int best_VLC_AP = 0;
        double max_data_rate = -1;

        for (int j = 0; j < VLC_AP_num; j++) {
            double total_link_data_rate = 0.0;

            for (int k = 0; k < effective_subcarrier_num; k++)
                total_link_data_rate += VLC_data_rate_matrix[j][i][k];
            total_link_data_rate *= time_slot_num;

            if (total_link_data_rate > max_data_rate) {
                max_data_rate = total_link_data_rate;
                best_VLC_AP = j+1;  // indexed starting from 1
            }
        }
        strategy_set[i][1] = best_VLC_AP;
    }


    // step2: UE are randomly allocated to an AP from their strategy set (APA)
    std::vector<int> serving_AP (UE_num, -1);
    std::vector<int> served_UE_num (RF_AP_num + VLC_AP_num, 0);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(0, 1);

    for (int i = 0; i < UE_num; i++) {
        int chosen_AP = strategy_set[distribution(generator)];

        serving_AP[i] = chosen_AP;
        served_UE_num[chosen_AP]++;
    }

    updateApAssociationResult(my_UE_list, serving_AP, AP_sssociation_matrix);

    //step3: resource allocation
    std::vector<double> throughput (UE_num, 0.0);

    // if connected to RF AP -> equally divide RF AP resource
    for (int i = 0; i < UE_num; i++) {
        if (serving_AP[i] == 0)
            throughput[i] = RF_data_rate_vector[served_UE_num[0]];
    }

    // if connected to VLC AP -> OFDMA
    for (int i = 0; i < VLC_AP_num; i++) {


    }
}

void algorithmExceptFirstState()
{

}



void updateApAssociationResult(std::vector<MyUeNode> &my_UE_list,
                               std::vector<int> &serving_AP,
                               std::vector<std::vector<int>> &AP_sssociation_matrix)
{
    // update every myUeNode
    for (int i = 0; i < UE_num; i++)
        my_UE_list[i].setCurrAssociatedAP(serving_AP[i]);

    // update the AP_sssociation_matrix
    for (int i = 0; i < AP_sssociation_matrix.size(); i++)
        for (int j = 0; j < AP_sssociation_matrix[0].size(); j++)
            AP_sssociation_matrix[i][j] = 0;

    for (int i = 0; i < my_UE_list.size(); i++)
        AP_sssociation_matrix[my_UE_list[i].getCurrAssociatedAP()][i] = 1;

}
