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
double getDistance(Ptr<Node> AP, MyUeNode &UE); // in meters
double getIrradianceAngle(Ptr<Node> AP, Ptr<Node> UE); // in radians
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP_node, Ptr<Node> UE_node, MyUeNode &UE);
double getRandomOrientation(MyUeNode &UE);


/*
    channel gain
*/
// VLC
// line of sight
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE);
double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes, std::vector<MyUeNode> &myUElist, std::vector<std::vector<double>> &VLC_LOS_matrix);
//front-end
double estimateOneVlcFrontEnd(int subcarrier_index);

/*
    SINR
*/
// VLC
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index);
void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix);

/*
    data rate
*/
// RF
double calculateRfSystemUtilization(int serving_num);
double calculateRfDownlinkUtilizationEfficiency(int serving_num);
void calculateAllRfDownlinkUtilizationEfficiency(std::vector<double> &downlink_utilization_efficiency);

// VLC
double getSpectralEfficiency(double SINR);
double estimateVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, int VLC_AP_index, int UE_index, int subcarrier_index);
void calculateAllVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix);


#endif // CHANNEL_H
