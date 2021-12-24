#ifndef CHANNEL_H
#define CHANNEL_H

#include "global_configuration.h"



using namespace ns3;

/*
    distance and angle calculation
*/
double radian2Degree(const double &radian);
double degree2Radian(const double &degree);
double getDistance(Ptr<Node> AP, Ptr<Node> UE); // in meters
double getIncidenceAngle(Ptr<Node> AP, Ptr<Node> UE); // in radians

/*
    channel gain
*/
double estimateOneRfChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE);
double estimateOneVlcChannelGain(Ptr<Node> VLC_AP, Ptr<Node> UE);
void calculateAllRfChannelGain(NodeContainer &RF_AP_nodes, NodeContainer &UE_nodes, std::vector<std::vector<double>> &RF_channel_gain_matrix);
void calculateAllVlcChannelGain(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes, std::vector<std::vector<double>> &VLC_channel_gain_matrix);

/*
    SINR
*/
double estimateOneRfSINR(std::vector<std::vector<double>> &RF_channel_gain_matrix, int RF_AP_index, int UE_index);
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_channel_gain_matrix, int VLC_AP_index, int UE_index);
void calculateAllRfSINR(std::vector<std::vector<double>> &RF_channel_gain_matrix, std::vector<std::vector<double>> &RF_SINR_matrix);
void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_channel_gain_matrix, std::vector<std::vector<double>> &VLC_SINR_matrix);

/*
    data rate
*/
void calculateRfDataRate(std::vector<std::vector<double>> &RF_SINR_matrix, std::vector<std::vector<double>> &RF_data_rate_matrix);
void calculateVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, std::vector<std::vector<double>> &VLC_data_rate_matrix);

/*
    bit error ratio
*/
void calculateBitErrorRatio(std::vector<std::vector<double>> &VLC_SINR_matrix, std::vector<double> &bit_error_ratios);


#endif // CHANNEL_H
