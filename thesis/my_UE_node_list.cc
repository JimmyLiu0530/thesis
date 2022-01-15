#include <iostream>
#include <fstream>
#include <string>
#include <chrono> // seed

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "My_UE_Node_List.h"


/*
 * create an myUeNode object for each UE
 */
std::vector<MyUeNode> initializeMyUeNodeList(NodeContainer &UE_nodes)
{
    std::vector<MyUeNode> my_UE_list;

    // first, randomly generate required data rate and orientation angle for each UE
    // then, instantiate an new myUeNode object and add it into my_UE_list

    for (int i = 0; i < UE_num; i++) {
        double required_data_rate = 0.0;

        Ptr<MobilityModel> UE_mobility_model = (UE_nodes.Get(i))->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        my_UE_list.push_back(MyUeNode(i, pos, required_data_rate, 0.0));
    }

    return my_UE_list;
}
