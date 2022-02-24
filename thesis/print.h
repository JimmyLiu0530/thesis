#ifndef PRINT_H
#define PRINT_H

#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void printVlcLosMatrix(std::vector<std::vector<double>> &VLC_LOS_matrix);

void printVlcSinrMatrix(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix);

void printRfDataRateVector(std::vector<double> &RF_data_rate_vector);

void printVlcDataRateMatrix(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix);

void printApAssociationMatrix(std::vector<std::vector<int>> &AP_association_matrix);

void printRfApPosition(ns3::NodeContainer &RF_AP_node);

void printVlcApPosition(ns3::NodeContainer &VLC_AP_nodes);

void printUePosition(ns3::NodeContainer &UE_nodes);

void printMyUeList(std::vector<MyUeNode> &my_UE_list);

void printResourceUnitMatrix(std::vector<std::vector<std::vector<int>>> &resource_unit_matrix, int VLC_AP_index);

void printBlockedUE(std::vector<int> &blocked_UE);
#endif
