#include <iostream>
#include <fstream>
#include <string>
#include <chrono> // seed

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"
#include "my_UE_node_list.h"


/*
 * create an myUeNode object for each UE
 */
std::vector<MyUeNode> initializeMyUeNodeList(NodeContainer &UE_nodes)
{
    std::vector<MyUeNode> my_UE_list;

    // first, randomly generate required data rate and orientation angle for each UE
    // then, instantiate an new myUeNode object and add it into my_UE_list

    std::uniform_real_distribution<double> uniform(40, 70);
    std::default_random_engine generator (std::chrono::system_clock::now().time_since_epoch().count());

    for (int i = 0; i < UE_num; i++) {
        double required_data_rate = uniform(generator);

        Ptr<MobilityModel> UE_mobility_model = (UE_nodes.Get(i))->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        my_UE_list.push_back(MyUeNode(i, pos, required_data_rate));
    }

    return my_UE_list;
}
