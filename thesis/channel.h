#ifndef CHANNEL_H
#define CHANNEL_H

#include <map>

#include "global_configuration.h"
#include "my_UE_node.h"


using namespace ns3;



/*
    distance and angle calculation
*/
double radian2Degree(const double &radian);
double degree2Radian(const double &degree);
double getDistance(Ptr<Node> AP, MyUeNode &UE_node); // in meters
double getIrradianceAngle(Ptr<Node> AP, Ptr<Node> UE); // in radians
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node);
double getRandomOrientation(MyUeNode &UE_node);


/*
 * a high-level function for calculating all channel-related information for VLC and RF,
 * including channel gain, SINR, achievable data rate
 */
void precalculation(NodeContainer &RF_AP_node ,
                      NodeContainer &VLC_AP_nodes ,
                      NodeContainer &UE_nodes,
                      std::vector<std::vector<double>> &VLC_LOS_matrix,
                      std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                      std::vector<double> &RF_data_rate_vector,
                      std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                      std::vector<MyUeNode> &my_UE_list);

/*
    channel gain
*/
// VLC
// line of sight
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node);
double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes, std::vector<MyUeNode> &my_UE_list, std::vector<std::vector<double>> &VLC_LOS_matrix);

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
double calculateRfSystemUtilization(int serving_UE_num);
double calculateRfDownlinkUtilizationEfficiency(int serving_UE_num);
void calculateRfDataRate(std::vector<double> &RF_data_rate_vector);

// VLC
double getSpectralEfficiency(double SINR);
double estimateVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, int VLC_AP_index, int UE_index, int subcarrier_index);
void calculateAllVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix);


#endif // CHANNEL_H
