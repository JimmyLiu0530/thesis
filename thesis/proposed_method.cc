#include <algorithm>
#include <limits>
#include <map>

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
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);

    if (state == 0)
        proposedMethodForState0(RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, resource_unit_matrix_per_VLC_AP, my_UE_list);
    else
        proposedMethodForStateN(RF_data_rate_vector, VLC_data_rate_matrix, AP_association_matrix, resource_unit_matrix_per_VLC_AP, my_UE_list);

}

void proposedMethodForState0(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<MyUeNode> &my_UE_list)
{
    // step1: find the VLC AP with the best SINR for each UE. If the max SINR is smaller than 1, then choose the RF AP.
    vector<int> best_SINR_AP (UE_num, -1);

    for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
        int best_AP = 0;
        double max_SINR = 0.0;
        for (int i = 1; i <= effective_subcarrier_num; i++)
            max_SINR += VLC_SINR_matrix[best_AP][UE_idx][i];

        for (int AP_idx = 1; AP_idx < VLC_AP_num; AP_idx++) {
            double total_SINR = 0.0;
            for (int i = 1; i <= effective_subcarrier_num; i++)
                total_SINR += VLC_SINR_matrix[AP_idx][UE_idx][i];

            if (total_SINR > max_SINR) {
                best_AP = AP_idx;
                max_SINR = total_SINR;
            }
        }
        // here means no VLC AP can provide data rate for this UE, so choose RF AP as its best AP
        if (max_SINR < 1)
            best_SINR_AP[UE_idx] = 0;
        else
            best_SINR_AP[UE_idx] = best_AP+1; // VLC APs are indexed from 1
    }


    // step2: sort UEs in the descending order based on their demands
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getRequiredDataRate() > b.getRequiredDataRate();});


    // step3: find the AP with max residual resource after fulfilling the user u's demand
    //        and choose it to be the user's serving AP.
    for (int UE_idx = 0; UE_idx < my_UE_list.size(); UE_idx++) {
        if (best_SINR_AP[my_UE_list[UE_idx].getID()] != 0) { // best AP is not RF AP
            std::vector<int> AP_order = sortApBasedOnResidualResource(resource_unit_matrix_per_VLC_AP, my_UE_list[UE_idx]);

        }
    }




}

void proposedMethodForStateN(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                             std::vector<double> &RF_data_rate_vector,
                             std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                             std::vector<std::vector<int>> &AP_association_matrix,
                             std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                             std::vector<MyUeNode> &my_UE_list)
{

}


// sort VLC APs in the descending order based on their residual resource after fulfilling UE's demand
std::vector<int> sortApBasedOnResidualResource(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                               std::vector<std::vector<std::vector<int>>> &resource_unit_matrix_per_VLC_AP,
                                               MyUeNode &UE)
{
    std::map<int,int,std::greater<int>> residual_RU_num; // {residual RU num, VLC AP index}
    std::vector<double> offered_data_rate_per_AP;

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
        std::vector<std::vector<int>> resource_unit_matrix = resource_unit_matrix_per_VLC_AP[VLC_AP_idx];

        int UE_idx = UE.getID();
        double UE_demand = UE.getRequiredDataRate();
        double offered_data_rate = 0.0;
        int subcarrier_idx = 1;
        int time_slot_idx = 0;

        // find the beginning of the resource_unit_matrix
        while (resource_unit_matrix[subcarrier_idx][time_slot_idx] == 1) {
            time_slot_idx++;

            if (time_slot_idx == time_slot_num) {
                time_slot_idx = 0;
                subcarrier_idx++;
            }
        }

        while ((offered_data_rate < UE_demand) && (subcarrier_idx <= effective_subcarrier_num) && (VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx] == 0.0)) {
            offered_data_rate += VLC_data_rate_matrix[VLC_AP_idx][UE_idx][subcarrier_idx];
            resource_unit_matrix[subcarrier_idx][time_slot_idx] = 1;
            time_slot_idx++;

            if (time_slot_idx == time_slot_num) {
                time_slot_idx = 0;
                subcarrier_idx++;
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
        // this AP does not fulfill the user's demand, then set residual RU number to 0
        else
            residual_RU_num[0] = VLC_AP_idx;
    }

    std::vector<int> result;
    auto it = residual_RU_num.begin();

    // for those APs that fulfill the demand, sort them by the number of residual RUs
    for (; it != residual_RU_num.end() && it->first > 0; it++) {
        result.push_back(it->second);
    }

    // for those APs that don't fulfill the demand, sort them by data rate they offer
    std::map<double,int,std::greater<double>> sorting;

    for (; it != residual_RU_num.end(); it++) {
        sorting[offered_data_rate_per_AP[it->second]] = it->second;
    }
    for (auto it2 = sorting.begin(); it2 != sorting.end(); it2++)
        result.push_back(it2->second);


    return result;
}

