#ifndef PRINT_H
#define PRINT_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void print_RF_Channel_Gain_Matrix(std::vector<std::vector<double>> &RF_Channel_Gain_Matrix);

void print_VLC_Channel_Gain_Matrix(std::vector<std::vector<double>> &VLC_Channel_Gain_Matrix);

void print_RF_SINR_Matrix(std::vector<std::vector<double>> &RF_SINR_Matrix);

void print_VLC_SINR_Matrix(std::vector<std::vector<double>> &VLC_SINR_Matrix);

void print_RF_DataRate_Matrix(std::vector<std::vector<double>> &RF_DataRate_Matrix);

void print_VLC_DataRate_Matrix(std::vector<std::vector<double>> &VLC_DataRate_Matrix);

void print_AP_Association_Matrix(std::vector<std::vector<int>> &AP_Association_Matrix);

void print_TDMA_Matrix(std::vector<std::vector<double>> &TDMA_Matrix);

void print_RF_AP_position(NodeContainer &RF_AP_Nodes);

void print_VLC_AP_position(NodeContainer &VLC_AP_Nodes);

void print_UE_position(NodeContainer &UE_Nodes);

#endif
