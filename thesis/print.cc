#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

#include "print.h"
#include "global_environment.h"
#include "My_UE_Node.h"

void print_RF_Channel_Gain_Matrix(std::vector<std::vector<double>> &RF_Channel_Gain_Matrix)
{
    std::cout << "RF Channel Gain Matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {
            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_Channel_Gain_Matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void print_VLC_Channel_Gain_Matrix(std::vector<std::vector<double>> &VLC_Channel_Gain_Matrix)
{
    std::cout << "VLC Channel Gain Matrix as below : " << std::endl;

    for (int i = 0; i < VLC_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {

            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << VLC_Channel_Gain_Matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void print_RF_SINR_Matrix(std::vector<std::vector<double>> &RF_SINR_Matrix)
{
    std::cout << "RF SINR Matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {
            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_SINR_Matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void print_VLC_SINR_Matrix(std::vector<std::vector<double>> &VLC_SINR_Matrix)
{
    std::cout << "VLC SINR Matrix as below : " << std::endl;

    for (int i = 0; i < VLC_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {

            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << VLC_SINR_Matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void print_RF_DataRate_Matrix(std::vector<std::vector<double>> &RF_DataRate_Matrix)
{

    std::cout << "RF DataRate Matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {

            std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2) << RF_DataRate_Matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void print_VLC_DataRate_Matrix(std::vector<std::vector<double>> &VLC_DataRate_Matrix)
{

    std::cout << "VLC DataRate Matrix as below : " << std::endl;

    for (int i = 0; i < VLC_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {

            //速度 < 1 的太小了 show出來沒意義
            //視爲 0
            if (VLC_DataRate_Matrix[i][j] > 1)

                std::cout << std::left << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(2) << VLC_DataRate_Matrix[i][j] << " ";
            else

                std::cout << std::left << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(2) << 0 << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}



void print_AP_Association_Matrix(std::vector<std::vector<int>> &AP_Association_Matrix)
{

    std::cout << "AP_Association_Matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_Num + VLC_AP_Num; i++)
    {

        for (int j = 0; j < UE_Num; j++)
        {

            std::cout << AP_Association_Matrix[i][j] << " ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}


void print_TDMA_Matrix(std::vector<std::vector<double>> &TDMA_Matrix)
{

    std::cout << "TDMA_Matrix as below : " << std::endl;

    for (int i = 0; i < RF_AP_Num + VLC_AP_Num; i++)
    {

        if (i < RF_AP_Num)
            std::cout << std::left << "RF AP " << std::setw(4) << i;
        else
            std::cout << std::left << "VLC AP " << std::setw(3) << i - RF_AP_Num;

        double sum = 0;
        for (int j = 0; j < UE_Num + 1; j++)
        {

            if (j > 0)
                sum += TDMA_Matrix[i][j];
            if (j == 0)
                std::cout << "Residual = ";
            std::cout << std::left << std::setw(6) << std::setiosflags(std::ios::fixed) << std::setprecision(3) << TDMA_Matrix[i][j] << " ";
        }

        std::cout << std::left << std::setw(6) << "total =" << sum << std::endl;
    }

    std::cout << std::endl;
}

void print_RF_AP_position(NodeContainer &RF_AP_Nodes) {
    int RF_AP_Index = 1;
    for (NodeContainer::Iterator it = RF_AP_Nodes.Begin(); it != RF_AP_Nodes.End(); ++it)
    {
        Ptr<MobilityModel> RF_MobilityModel = (*it)->GetObject<MobilityModel>();
        Vector pos = RF_MobilityModel->GetPosition();
        std::cout << "Position of RF_AP " << RF_AP_Index++ << " =(" << pos.x << "," << pos.y << "," << pos.z << ")" << std::endl;
    }
    std::cout << std::endl;
}

void print_VLC_AP_position(NodeContainer &VLC_AP_Nodes) {
    int VLC_AP_Index = 1;
    for (NodeContainer::Iterator it = VLC_AP_Nodes.Begin(); it != VLC_AP_Nodes.End(); ++it)
    {
        Ptr<MobilityModel> VLC_MobilityModel = (*it)->GetObject<MobilityModel>();
        Vector pos = VLC_MobilityModel->GetPosition();
        std::cout << "Position of VLC_AP " << VLC_AP_Index++ << " =(" << pos.x << "," << pos.y << "," << pos.z << ")" << std::endl;
    }
    std::cout << std::endl;
}

void print_UE_position(NodeContainer &UE_Nodes) {
    int UE_Index = 1;
    for (NodeContainer::Iterator it = UE_Nodes.Begin(); it != UE_Nodes.End(); ++it)
    {
        Ptr<MobilityModel> UE_MobilityModel = (*it)->GetObject<MobilityModel>();
        Vector pos = UE_MobilityModel->GetPosition();
        std::cout << "Position of UE " << UE_Index++ << " =(" << pos.x << "," << pos.y << "," << pos.z << ")" << std::endl;
    }
    std::cout << std::endl;
}

