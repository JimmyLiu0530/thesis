#ifndef PRINT_H
#define PRINT_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void printRfChannelGainMatrix(std::vector<std::vector<double>> &RF_channel_gain_matrix);

void printVlcLosMatrix(std::vector<std::vector<double>> &VLC_LOS_matrix);

void printRfSinrMatrix(std::vector<std::vector<double>> &RF_SINR_matrix);

void printVlcSinrMatrix(std::vector<std::vector<double>> &VLC_SINR_matrix);

void printRfDataRateVector(std::vector<double> &RF_data_rate_vector);

void printVlcDataRateMatrix(std::vector<std::vector<double>> &VLC_data_rate_matrix);

void printApAssociationMatrix(std::vector<std::vector<int>> &AP_association_matrix);

void printTdmaMatrix(std::vector<std::vector<double>> &TDMA_matrix);

void printRfApPosition(NodeContainer &RF_AP_node);

void printVlcApPosition(NodeContainer &VLC_AP_nodes);

void printUePosition(NodeContainer &UE_nodes);

#endif
