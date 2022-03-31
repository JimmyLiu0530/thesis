#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>

#include "print.h"
#include "global_configuration.h"
#include "my_UE_node.h"


void printVlcLosMatrix(std::vector<std::vector<double>> &VLC_LOS_matrix)
{
    std::cout << "VLC LOS matrix as below: " << std::endl;

    for (int i = 0; i < VLC_AP_num; i++) {
        for (int j = 0; j < UE_num; j++) {
            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(15) << VLC_LOS_matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void printVlcSinrMatrix(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix)
{
    std::cout << "VLC SINR matrix as below: " << std::endl;

    for (int i = 0; i < VLC_AP_num; i++) {
        std::cout << "For VLC AP " << i << ": \n";

        for (int j = 0; j < UE_num; j++) {
            std::cout << "\tFor UE " << j << ": \n";
            std::cout << "\t";

            for (int k = 0; k < subcarrier_num; k++) {
                std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(4) << VLC_SINR_matrix[i][j][k] << " ";
            }

            std::cout << std::endl;
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}

void printRfDataRateVector(std::vector<double> &RF_data_rate_vector)
{

    std::cout << "RF data rate vector for different number of serving UE as below : " << std::endl;

    for (int i = 0; i < UE_num+1; i++) {
        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_data_rate_vector[i] << " ";
    }

    std::cout << std::endl << std::endl;
}

void printVlcDataRateMatrix(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix)
{
    std::cout << "VLC data rate matrix as below: " << std::endl;

    for (int i = 0; i < VLC_AP_num; i++) {
        std::cout << "For VLC AP " << i << ": \n";

        for (int j = 0; j < UE_num; j++) {
            std::cout << "\tFor UE " << j << ": \n";
            std::cout << "\t";

            for (int k = 0; k < subcarrier_num; k++) {
                std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(4) << VLC_data_rate_matrix[i][j][k] << " ";
            }

            std::cout << std::endl;
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}

void printApAssociationMatrix(std::vector<std::vector<int>> &AP_association_matrix)
{

    std::cout << "AP association matrix as below: " << std::endl;

    for (int i = 0; i < RF_AP_num + VLC_AP_num; i++) {
        for (int j = 0; j < UE_num; j++) {
            std::cout << AP_association_matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}

void printRfApPosition(ns3::NodeContainer &RF_AP_node) {
    int RF_AP_index = 1;

    for (NodeContainer::Iterator it = RF_AP_node.Begin(); it != RF_AP_node.End(); ++it) {
        Ptr<MobilityModel> RF_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector pos = RF_mobility_model->GetPosition();

        std::cout << "Position of RF_AP " << RF_AP_index++ << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    std::cout << std::endl;
}

void printVlcApPosition(ns3::NodeContainer &VLC_AP_nodes) {
    int VLC_AP_index = 1;

    std::fstream output;
    output.open("/home/hsnl/repos/ns-3-allinone/ns-3.25/scratch/thesis/VLC_AP_position.csv", std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
    }

    for (NodeContainer::Iterator it = VLC_AP_nodes.Begin(); it != VLC_AP_nodes.End(); ++it) {
        Ptr<MobilityModel> VLC_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector pos = VLC_mobility_model->GetPosition();

        output << pos.x << "," << pos.y << ",";
        output << std::endl;

        std::cout << "Position of VLC_AP " << VLC_AP_index++ << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    std::cout << std::endl;
}

void printUePosition(ns3::NodeContainer &UE_nodes) {
    int UE_index = 0;

    std::fstream output;
    output.open("/home/hsnl/repos/ns-3-allinone/ns-3.25/scratch/thesis/UE_position.csv", std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }


    for (NodeContainer::Iterator it = UE_nodes.Begin(); it != UE_nodes.End(); ++it) {
        Ptr<MobilityModel> UE_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        output << pos.x << "," << pos.y << ",";
        output << std::endl;

        std::cout << "Position of UE " << UE_index++ << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    output << std::endl;

    std::cout << std::endl;
}

void printUePosition(std::vector<MyUeNode> &my_UE_list) {

    std::fstream output;
    output.open("/home/hsnl/repos/ns-3-allinone/ns-3.25/scratch/thesis/UE_position.csv", std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < my_UE_list.size(); i++) {
        Vector pos = my_UE_list[i].getPosition();

        output << pos.x << "," << pos.y << ",";
        output << std::endl;

        std::cout << "Position of UE " << my_UE_list[i].getID() << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }

    output << std::endl;

    std::cout << std::endl;
}

void printMyUeList(std::vector<MyUeNode> &my_UE_list) {
    for (int i = 0; i < my_UE_list.size(); i++) {
        std::cout << "node id: " << my_UE_list[i].getID() << std::endl;

        Vector pos = my_UE_list[i].getPosition();
        std::cout << "position is (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        std::cout << "demand is " << my_UE_list[i].getRequiredDataRate() << std::endl;
        std::cout << "polar angle is " << my_UE_list[i].getPolarAngle() << std::endl;
        std::cout << "azimuth angle is " << my_UE_list[i].getAzimuthAngle() << std::endl << std::endl;
    }
}

void printResourceUnitMatrix(std::vector<std::vector<std::vector<int>>> &resource_unit_matrix, int VLC_AP_index) {
    std::cout << "RU Matrix of VLC AP " << VLC_AP_index << " as below: " << std::endl;

    for (int i = 0; i < resource_unit_matrix[VLC_AP_index].size(); i++) {
        for (int j = 0; j < resource_unit_matrix[VLC_AP_index][i].size(); j++) {
            std::cout << std::setw(4) << resource_unit_matrix[VLC_AP_index][i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void printBlockedUE(std::vector<int> &blocked_UE) {
    std::cout << "Blocked UE as below: " << std::endl;

    std::sort(blocked_UE.begin(), blocked_UE.end());

    for (int i = 0; i < blocked_UE.size(); i++)
        std::cout << blocked_UE[i] << " ";

    std::cout << std::endl;
}
