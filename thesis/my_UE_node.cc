#include <iostream>
#include <fstream>
#include <string>


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"


using namespace ns3;


MyUeNode::MyUeNode(int node_ID, Vector pos, double required_data_rate, double orientation_angle)
    : generator(std::chrono::system_clock::now().time_since_epoch().count()), distribution(0.0, sqrt(noise_variance))
    {
        this.node_ID = node_ID;
        this.pos = pos;
        this.required_data_rate = required_rate;
        this.orientation_angle = orientation_angle;

        avg_throughput = 0;
        prev_associated_AP = -1;
        cur_associated_AP = -1;
        SINR = 0;
    }

int My_UE_Node::getID(void) {
    reutrn node_id;
}

void My_UE_Node::setPosition(Vector pos_from_mobility_model) {
    pos = pos_from_mobility_model;
}

Vector My_UE_Node::getPosition(void) {
    return pos;
}

void My_UE_Node::setRequiredDataRate(double data_rate_in_Mbps) {
    required_data_rate = data_rate_in_Mbps;
}

double My_UE_Node::getRequiredDataRate(void) {
    return required_data_rate;
}

void My_UE_Node::setAvgThroughput(double data_rate_in_Mbps) {
    avg_throughput = data_rate_in_Mbps;
}

double My_UE_Node::getAvgThroughput(void) {
    return avg_throughput;
}

void My_UE_Node::setCurrAssociatedAP(int associated_AP_index) {
    prev_associated_AP = cur_associated_AP;
    cur_associated_AP = associated_AP_index;
}

int My_UE_Node::getCurrAssociatedAP(void) {
    return cur_associated_AP;
}

int My_UE_Node::getPrevAssociatedAP(void) {
    return prev_associated_AP;
}

void My_UE_Node::setSINR(double in_SINR) {
    SINR = in_SINR;
}

double My_UE_Node::getSINR(void) {
    return SINR;
}

void My_UE_Node::addCurrIterationThroughput(double data_rate_in_Mbps) {
    throughput_per_iteration.push_back(data_rate_in_Mbps);

    double sum = 0;
    for (int i = 0; i < throughput_per_iteration.size(); i++)
        sum += throughput_per_iteration[i];

    setAvgThroughput(sum / throughput_per_iteration.size());

}
std::vector<double> My_UE_Node::getThroughputHistory(void) {
    return throughput_per_iteration;
}

void My_UE_Node::addCurrSatisfactionLevel(double satis_level) {
    satisfication_level_per_iteration.push_back(satis_level);

}
std::vector<double> My_UE_Node::getSatisfactionLevelHistory(void) {
    return satisfication_level_per_iteration;
}

double getOrientationAngle(void) {
    return orientation_angle;
}

void randomAnOrientationAngle(void) {
    double new_angle = c_0 + c_1 * orientation_angle + distribution(generator);

    orientation_angle = new_angle;
}












