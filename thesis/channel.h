#ifndef CHANNEL_H
#define CHANNEL_H

#include <map>

#include "global_configuration.h"


using namespace ns3;


/*
    table of conversion from SINR to spectral efficiency
*/
const std::map<double, double> SINR_to_spectral_efficiency;
SINR_to_spectral_efficiency[1] = 0.877;
SINR_to_spectral_efficiency[3] = 1.1758;
SINR_to_spectral_efficiency[5] = 1.4766;
SINR_to_spectral_efficiency[8] = 1.9141;
SINR_to_spectral_efficiency[9] = 2.4063;
SINR_to_spectral_efficiency[11] = 2.7305;
SINR_to_spectral_efficiency[12] = 3.3223;
SINR_to_spectral_efficiency[14] = 3.9023;
SINR_to_spectral_efficiency[16] = 4.5234;
SINR_to_spectral_efficiency[18] = 5.1152;
SINR_to_spectral_efficiency[20] = 5.5547;


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
// RF
double estimateOneRfChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE);
void calculateAllRfChannelGain(NodeContainer &RF_AP_nodes, NodeContainer &UE_nodes, std::vector<std::vector<double>> &RF_channel_gain_matrix);
// VLC
// LOS of all channels can be calculated in advance
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, double frequecny);
double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes, std::vector<std::vector<double>> &VLC_LOS_matrix);

double estimateOneVlcFrontEnd(int subcarrier_index);

/*
    SINR
*/
// RF
double estimateOneRfSINR(std::vector<std::vector<double>> &RF_channel_gain_matrix, int RF_AP_index, int UE_index);
void calculateAllRfSINR(std::vector<std::vector<double>> &RF_channel_gain_matrix, std::vector<std::vector<double>> &RF_SINR_matrix);
// VLC
// Because SINR here is dependent on subcarrier frequency,
// not able to calculate all SINR at a time (unless to consider all possible frequencies)
double calculateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index);


/*
    data rate
*/
void calculateRfDataRate(std::vector<std::vector<double>> &RF_SINR_matrix, std::vector<std::vector<double>> &RF_data_rate_matrix);
double getSpectralEfficiency(double SINR);
void calculateVlcDataRate(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index);



#endif // CHANNEL_H
