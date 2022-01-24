#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>


#include "benchmark.h"
#include "channel.h"
#include "my_UE_node.h"
#include "print.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void benchmarkDynamicLB(int &state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                       std::vector<std::vector<int>> &AP_association_matrix,
                       std::vector<MyUeNode> &my_UE_list)
{
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,
                   RF_data_rate_vector, VLC_data_rate_matrix, my_UE_list);


    std::vector<int> prev_serving_AP = initializeStep(RF_data_rate_vector,
                                                       VLC_data_rate_matrix,
                                                       AP_association_matrix,
                                                       my_UE_list);

    int next_iteration_flag = 0;
    int iteration_cnt = 1;

    do
    {
        next_iteration_flag = 0;
        std::vector<int> new_serving_AP = EGT_basedLoadBalance(RF_data_rate_vector,
                                                               VLC_data_rate_matrix,
                                                               AP_association_matrix,
                                                               my_UE_list);

        // check if no AP switch occurs
        for (int i = 0; i < UE_num; i++) {
            if (prev_serving_AP[i] != new_serving_AP[i]) {
                next_iteration_flag = 1;
                prev_serving_AP = new_serving_AP;
                iteration_cnt++;
                break;
            }
        }
    } while (next_iteration_flag);


#if DEBUG_MODE
    std::cout << "State " << state << " takes " << iteration_cnt << " iteration(s) to converge\n";

    for (int i = 0; i < my_UE_list.size(); i++) {
        std::cout << "History data rate of UE id: " << my_UE_list[i].getID() << std::endl;
        std::vector<double> history = my_UE_list[i].getThroughputHistory();

        for (int j = 0; j < history.size(); j++)
            std::cout << history[j] << " ";
        std::cout << std::endl;
    }
    std::cout<<std::endl;
#endif // DEBUG_MODE

    state++;
}

std::vector<int> initializeStep(std::vector<double> &RF_data_rate_vector,
                                std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                std::vector<std::vector<int>> &AP_association_matrix,
                                std::vector<MyUeNode> &my_UE_list)
{
    // step1: construct strategy sets of the form {RF_AP, best VLC_AP} for all UEs
    std::vector<std::vector<int>> strategy_set (UE_num, std::vector<int> (2, -1)); // {{RF_AP_idx, best VLC_AP_idx}, {RF_AP_idx, best VLC_AP_idx}, ...}

    for (int i = 0; i < UE_num; i++) {
        strategy_set[i][0] = 0; // strategy set always contains the RF AP

        int best_VLC_AP = 0;
        double max_data_rate = -1;

        for (int j = 0; j < VLC_AP_num; j++) {
            double total_link_data_rate = 0.0;

            for (int k = 1; k < effective_subcarrier_num + 1; k++)
                total_link_data_rate += VLC_data_rate_matrix[j][i][k];

            total_link_data_rate *= time_slot_num;

            if (total_link_data_rate > max_data_rate) {
                max_data_rate = total_link_data_rate;
                best_VLC_AP = j + RF_AP_num;  // VLC APs are indexed starting from 1
            }
        }
        strategy_set[i][1] = best_VLC_AP;
    }

    // step2: UE are randomly allocated to an AP from their strategy set (APA)
    std::vector<int> serving_AP (UE_num, -1);
    std::vector<std::vector<int>> served_UE (RF_AP_num + VLC_AP_num, std::vector<int> ()); // record the UEs served by a AP

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(0, 1);

    for (int i = 0; i < UE_num; i++) {
        int chosen_AP = (strategy_set[i][1] != 1) ? strategy_set[i][1] : 0;
        //int chosen_AP = strategy_set[i][distribution(generator)];

        serving_AP[i] = chosen_AP;
        served_UE[chosen_AP].push_back(i);
    }

    updateApAssociationResult(my_UE_list, serving_AP, AP_association_matrix);


    //step3: resource allocation
    std::vector<double> throughput (UE_num, 0.0);

    // if connected to RF AP -> equally divide RF AP resource
    for (int i = 0; i < RF_AP_num; i++) {
        for (int j = 0; j < served_UE[i].size(); j++) {
            throughput[served_UE[i][j]] = RF_data_rate_vector[served_UE[i].size()];
        }
    }

    // if connected to VLC AP -> OFDMA
    for (int i = RF_AP_num; i < VLC_AP_num + RF_AP_num; i++) {
        std::vector<double> data_rate = OFDMA(i-RF_AP_num, served_UE[i], VLC_data_rate_matrix);

        for (int j = 0; j < served_UE[i].size(); j++)
            throughput[served_UE[i][j]] = data_rate[j];
    }

    updateResourceAllocation(my_UE_list, throughput);

    return serving_AP;
}


std::vector<int> EGT_basedLoadBalance(std::vector<double> &RF_data_rate_vector,
                                        std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                        std::vector<std::vector<int>> &AP_association_matrix,
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

            for (int k = 1; k < effective_subcarrier_num + 1; k++)
                total_link_data_rate += VLC_data_rate_matrix[j][i][k];
            total_link_data_rate *= time_slot_num;

            if (total_link_data_rate > max_data_rate) {
                max_data_rate = total_link_data_rate;
                best_VLC_AP = j + RF_AP_num;  // indexed starting from 1 (i.e. the number of RF APs)
            }
        }
        strategy_set[i][1] = best_VLC_AP;
    }

    // step2: to see UE by UE whether it needs to mutate (change to another AP)
    double avg_payoff = 0.0;
    for (int i = 0; i < UE_num; i++) {
        avg_payoff += my_UE_list[i].getLastThroughput();
    }
    avg_payoff = avg_payoff / UE_num;


    std::vector<int> serving_AP (UE_num, -1);

    for (int i = 0; i < UE_num; i++) {
        double last_throughput = my_UE_list[i].getLastThroughput();
        double probability_of_mutation = (last_throughput < avg_payoff) ? (1 - last_throughput/avg_payoff) : 0;

        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        double random_number = distribution(generator);

        if (random_number < probability_of_mutation) { // mutation occurs
            if (my_UE_list[i].getCurrAssociatedAP() == 0) { // previous AP is RF AP
                std::vector<int> prev_served_UE = constructServedUeSet(AP_association_matrix, strategy_set[i][1]);

                prev_served_UE.push_back(i);
                std::vector<double> estimated_payoff_vec = OFDMA(strategy_set[i][1]-RF_AP_num, prev_served_UE, VLC_data_rate_matrix);

                double estimated_payoff = estimated_payoff_vec.back() * VHO_efficiency;

                if (estimated_payoff > my_UE_list[i].getLastThroughput())
                    serving_AP[i] = strategy_set[i][1];
                else
                    serving_AP[i] = my_UE_list[i].getCurrAssociatedAP();
            }
            else { // previous AP is VLC AP
                std::vector<int> prev_served_UE = constructServedUeSet(AP_association_matrix, strategy_set[i][0]);
                double estimated_payoff = RF_data_rate_vector[prev_served_UE.size()+1];

                if (estimated_payoff > my_UE_list[i].getLastThroughput())
                    serving_AP[i] = strategy_set[i][0];
                else
                    serving_AP[i] = my_UE_list[i].getCurrAssociatedAP();
            }
        }
        else { // no mutation occurs
            serving_AP[i] = my_UE_list[i].getCurrAssociatedAP();
        }
    }

    updateApAssociationResult(my_UE_list, serving_AP, AP_association_matrix);


    // step3: Resource allocation
    std::vector<double> throughput (UE_num, 0.0);
    std::vector<std::vector<int>> served_UE (RF_AP_num + VLC_AP_num, std::vector<int> ());

    for (int i = 0; i < RF_AP_num + VLC_AP_num; i++)
        served_UE[i] = constructServedUeSet(AP_association_matrix, i);


    // if connected to RF AP -> equally divide RF AP resource
    for (int i = 0; i < RF_AP_num; i++) {
        for (int j = 0; j < served_UE[i].size(); j++) {
            throughput[served_UE[i][j]] = RF_data_rate_vector[served_UE[i].size()];
        }
    }

    // if connected to VLC AP -> OFDMA
    for (int i = RF_AP_num; i < VLC_AP_num + RF_AP_num; i++) {
        std::vector<double> data_rate = OFDMA(i, served_UE[i], VLC_data_rate_matrix);

        for (int j = 0; j < served_UE[i].size(); j++)
            throughput[served_UE[i][j]] = data_rate[j];
    }

    updateResourceAllocation(my_UE_list, throughput);

    return serving_AP;
}

std::vector<int> constructServedUeSet(std::vector<std::vector<int>> &AP_association_matrix, int AP_index)
{
    std::vector<int> result;

    for (int i = 0; i < UE_num; i++) {
        if (AP_association_matrix[AP_index][i] == 1)
            result.push_back(i);
    }

    return result;
}


std::vector<double> OFDMA(int VLC_AP_index, std::vector<int> served_UE, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix)
{
    if (served_UE.empty())
        return {};

    int m = subcarrier_num / 2 - 1;
    std::vector<double> aggregate_data_rate (served_UE.size(), 0.0); // Z
    std::vector<std::vector<double>> subcarriers (served_UE.size(), std::vector<double> (m+1, 0.0)); // k
    std::vector<double> output_data_rate (served_UE.size(), 0.0);


    while (m >= 1) {
        for (int i = 0; i < served_UE.size(); i++) {
            if (m+1 > subcarrier_num / 2 - 1)
                aggregate_data_rate[i] = 0.0;
            else
                aggregate_data_rate[i] += subcarriers[i][m+1] * VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m+1];
        }

        double sum_data_rate = 0.0; // denominator of the first term in (33)
        double aggregate_data_rate_to_data_rate_sum = 0.0; // the second term of the second term in (33)
        for (int i = 0; i < served_UE.size(); i++) {
            sum_data_rate += pow(VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m], 1/beta - 1);

            if (VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m] == 0)
                continue;

            aggregate_data_rate_to_data_rate_sum += aggregate_data_rate[i] / VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m];
        }

        for (int i = 0; i < served_UE.size(); i++) {
            double first_term = pow(VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m], 1/beta - 1) / sum_data_rate;
            double second_term = time_slot_num + aggregate_data_rate_to_data_rate_sum;
            double last_term = (VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m] != 0) ? aggregate_data_rate[i] / VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][m] : 0;

            subcarriers[i][m] = first_term * second_term - last_term;
        }

        m = m - 1;
    }

    for (int i = 0; i < served_UE.size(); i++)
        for (int j = 1; j < subcarrier_num / 2; j++)
            output_data_rate[i] += subcarriers[i][j] * VLC_data_rate_matrix[VLC_AP_index][served_UE[i]][j];


    return output_data_rate;
}


void updateApAssociationResult(std::vector<MyUeNode> &my_UE_list,
                               std::vector<int> &serving_AP,
                               std::vector<std::vector<int>> &AP_sssociation_matrix)
{
    // update every myUeNode
    for (int i = 0; i < UE_num; i++)
        my_UE_list[i].setCurrAssociatedAP(serving_AP[i]);

    // update the AP association matrix
    // clear
    for (int i = 0; i < AP_sssociation_matrix.size(); i++)
        for (int j = 0; j < AP_sssociation_matrix[0].size(); j++)
            AP_sssociation_matrix[i][j] = 0;
    // assign
    for (int i = 0; i < UE_num; i++)
        AP_sssociation_matrix[my_UE_list[i].getCurrAssociatedAP()][i] = 1;

}


void updateResourceAllocation(std::vector<MyUeNode> &my_UE_list, std::vector<double> &throughput)
{
    for (int i = 0; i < my_UE_list.size(); i++) {
        my_UE_list[i].addThroughput(throughput[i]);

        // update satisfaction
        double curr_satisfaction = throughput[i] / my_UE_list[i].getRequiredDataRate();

        if (curr_satisfaction > 1)
            curr_satisfaction = 1;

        my_UE_list[i].addSatisfaction(curr_satisfaction);
    }
}
