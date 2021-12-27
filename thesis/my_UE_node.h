#ifndef MY_UE_NODE_H
#define MY_UE_NODE_H

#include <random>
#include <chrono>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_environment.h"


using namespace ns3;


class MyUeNode
{

public:
    MyUeNode(int node_ID, Vector pos, double required_data_rate, double orientation_angle);

    int getID(void);

    void setPosition(Vector pos_from_mobility_model);
    Vector getPosition(void);

    void setRequiredDataRate(double data_rate_in_Mbps);
    double getRequiredDataRate(void);

    void setAvgThroughput(double data_rate_in_Mbps);
    double getAvgThroughput(void);

    void setCurrAssociatedAP(int associated_AP_index);s
    int getCurrAssociatedAP(void);

    int getPrevAssociatedAP(void);

    void setSINR(double in_SINR);
    double getSINR(void);

    void addCurrIterationThroughput(double data_rate_in_Mbps);
    std::vector<double> getThroughputHistory(void);

    void addCurrSatisfactionLevel(double satis_level);
    std::vector<double> getSatisfactionLevelHistory(void);

    double getOrientationAngle(void);
    void randomAnOrientationAngle(void);

private:
    int node_ID;
    Vector pos;
    double required_data_rate;
    double avg_throughput;
    double orientation_angle;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

    // 0 indicates RF AP, and [1, VLC_AP_num] indicates VLC APs
    int prev_associated_AP;
    int curr_associated_AP;
    double SINR;
    std::vector<double> throughput_per_iteration;
    std::vector<double> satisfication_level_per_iteration;
};

#endif // MY_UE_NODE_H

