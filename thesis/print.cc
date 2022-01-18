#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

#include "print.h"
#include "global_configuration.h"
#include "my_UE_node.h"

void printRfChannelGainMatrix(std::vector<std::vector<double>> &RF_channel_gain_matrix)
{
    std::cout << "RF channel gain matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_num; i++)
    {

        for (int j = 0; j < UE_num; j++)
        {
            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_channel_gain_matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void printVlcLosMatrix(std::vector<std::vector<double>> &VLC_LOS_matrix)
{
    std::cout << "VLC channel gain matrix as below : " << std::endl;

    for (int i = 0; i < VLC_AP_num; i++)
    {

        for (int j = 0; j < UE_num; j++)
        {

            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << VLC_LOS_matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void printRfSinrMatrix(std::vector<std::vector<double>> &RF_SINR_matrix)
{
    std::cout << "RF SINR matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_num; i++)
    {

        for (int j = 0; j < UE_num; j++)
        {
            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_SINR_matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void printVlcSinrMatrix(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix)
{
    std::cout << "VLC SINR matrix as below: " << std::endl;

    for (int i = 0; i < VLC_AP_num; i++)
    {
        std::cout << "For VLC AP " << i << ": \n";
        for (int j = 0; j < UE_num; j++)
        {
            std::cout << "\tFor UE " << j << ": \n";
            std::cout << "\t\t";
            for (int k = 0; k < subcarrier_num; k++)
            {

                std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << VLC_SINR_matrix[i][j][k] << " ";
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


    for (int i = 0; i < UE_num+1; i++)
    {

        std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_data_rate_vector[i] << " ";
    }

    std::cout << std::endl;
}


void printVlcDataRateMatrix(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix)
{

    std::cout << "VLC data rate matrix as below : " << std::endl;

    for (int i = 0; i < VLC_AP_num; i++)
    {
        std::cout << "For VLC AP " << i << ": \n";
        for (int j = 0; j < UE_num; j++)
        {
            std::cout << "\tFor UE " << j << ": \n";
            std::cout << "\t\t";
            for (int k = 0; k < subcarrier_num; k++)
            {
                //速度 < 1 的太小了 show出來沒意義
                //視爲 0
                if (VLC_data_rate_matrix[i][j][k] > 1.0)
                    std::cout << std::left << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(2) << VLC_data_rate_matrix[i][j][k] << " ";

                else
                    std::cout << std::left << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(2) << 0 << " ";

            }

            std::cout << std::endl;
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}



void printApAssociationMatrix(std::vector<std::vector<int>> &AP_association_matrix)
{

    std::cout << "AP_association_matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_num + VLC_AP_num; i++)
    {

        for (int j = 0; j < UE_num; j++)
        {

            std::cout << AP_association_matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void printTdmaMatrix(std::vector<std::vector<double>> &TDMA_matrix)
{

    std::cout << "TDMA_matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_num + VLC_AP_num; i++)
    {

        if (i < RF_AP_num)
            std::cout << std::left << "RF AP " << std::setw(4) << i;
        else
            std::cout << std::left << "VLC AP " << std::setw(3) << i - RF_AP_num;

        double sum = 0;
        for (int j = 0; j < UE_num + 1; j++)
        {

            if (j > 0)
                sum += TDMA_matrix[i][j];
            if (j == 0)
                std::cout << "Residual = ";
            std::cout << std::left << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(3) << TDMA_matrix[i][j] << " ";
        }

        std::cout << std::left << std::setw(6) << "total =" << sum << std::endl;
    }

    std::cout << std::endl;
}

void printRfApPosition(ns3::NodeContainer &RF_AP_node) {
    int RF_AP_index = 1;

    for (NodeContainer::Iterator it = RF_AP_node.Begin(); it != RF_AP_node.End(); ++it)
    {
        Ptr<MobilityModel> RF_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector pos = RF_mobility_model->GetPosition();

        std::cout << "Position of RF_AP " << RF_AP_index++ << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    std::cout << std::endl;
}

void printVlcApPosition(ns3::NodeContainer &VLC_AP_nodes) {
    int VLC_AP_index = 1;

    for (NodeContainer::Iterator it = VLC_AP_nodes.Begin(); it != VLC_AP_nodes.End(); ++it)
    {
        Ptr<MobilityModel> VLC_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector pos = VLC_mobility_model->GetPosition();

        std::cout << "Position of VLC_AP " << VLC_AP_index++ << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    std::cout << std::endl;
}

void printUePosition(ns3::NodeContainer &UE_nodes) {
    int UE_index = 1;

    for (NodeContainer::Iterator it = UE_nodes.Begin(); it != UE_nodes.End(); ++it)
    {
        Ptr<MobilityModel> UE_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        std::cout << "Position of UE " << UE_index++ << " =(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    std::cout << std::endl;
}

void printMyUeList(std::vector<MyUeNode> &my_UE_list) {
    for (int i = 0; i < my_UE_list.size(); i++) {
        std::cout << "node id: " << my_UE_list[i].getID() << std::endl;
        Vector pos = my_UE_list[i].getPosition();
        std::cout << "position is (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        std::cout << "demand is " << my_UE_list[i].getRequiredDataRate() << std::endl;
        std::cout << "orientation angle is " << my_UE_list[i].getOrientationAngle() << std::endl << std::endl;
    }
}

