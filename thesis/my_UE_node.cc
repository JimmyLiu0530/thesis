#include <iostream>
#include <fstream>
#include <string>
#include <chrono>


#include "My_UE_Node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_environment.h"


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

int MyUeNode::getID(void) {
    reutrn node_id;
}

void MyUeNode::setPosition(Vector pos_from_mobility_model) {
    pos = pos_from_mobility_model;
}

Vector MyUeNode::getPosition(void) {
    return pos;
}

void MyUeNode::setRequiredDataRate(double data_rate_in_Mbps) {
    required_data_rate = data_rate_in_Mbps;
}

double MyUeNode::getRequiredDataRate(void) {
    return required_data_rate;
}

void MyUeNode::setAvgThroughput(double data_rate_in_Mbps) {
    avg_throughput = data_rate_in_Mbps;
}

double MyUeNode::getAvgThroughput(void) {
    return avg_throughput;
}

double MyUeNode::getLastThroughput(void) {
    return throughput_per_iteration.back();
}

void MyUeNode::setCurrAssociatedAP(int associated_AP_index) {
    prev_associated_AP = cur_associated_AP;
    cur_associated_AP = associated_AP_index;
}

int MyUeNode::getCurrAssociatedAP(void) {
    return cur_associated_AP;
}

int MyUeNode::getPrevAssociatedAP(void) {
    return prev_associated_AP;
}

void MyUeNode::setSINR(double in_SINR) {
    SINR = in_SINR;
}

double MyUeNode::getSINR(void) {
    return SINR;
}

void MyUeNode::addThroughput(double data_rate_in_Mbps) {
    throughput_per_iteration.push_back(data_rate_in_Mbps);

    // update avgerage throughput
    double sum = 0;
    for (int i = 0; i < throughput_per_iteration.size(); i++)
        sum += throughput_per_iteration[i];

    setAvgThroughput(sum / throughput_per_iteration.size());
}

std::vector<double> MyUeNode::getThroughputHistory(void) {
    return throughput_per_iteration;
}

void MyUeNode::addSatisfaction(double satis_level) {
    satisfaction_per_iteration.push_back(satis_level);

}
std::vector<double> MyUeNode::getSatisfactionHistory(void) {
    return satisfaction_per_iteration;
}

double MyUeNode::calculateAvgSatisfaction(void) {
    double satis_sum = 0.0;

    for (int i = 0; i < satisfaction_per_iteration.size(); i++) {
        satis_sum += satisfaction_per_iteration[i];
    }

    return satis_sum / satisfaction_per_iteration.size();
}

double MyUeNode::getOrientationAngle(void) {
    return orientation_angle;
}

void MyUeNode::randomAnOrientationAngle(void) {
    double new_angle = c_0 + c_1 * orientation_angle + distribution(generator); // based on (22) in "Realistic ..." paper

    orientation_angle = new_angle;
}












