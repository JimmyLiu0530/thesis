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

    std::default_random_engine generator (std::chrono::system_clock::now().time_since_epoch().count());

    std::uniform_int_distribution<int> uniform(1, 100);
    // std::uniform_real_distribution<double> uniform(1, 150);
    // std::normal_distribution<double> normal (75.5, 18.625);

    /*std::fstream input;
    input.open("/home/hsnl/repos/ns-3-allinone/ns-3.25/scratch/thesis/demand.txt", std::ios::in);

    if (!input) {
        std::cout << "fail to open demand.txt\n";
        exit(EXIT_FAILURE);
    }*/

    for (int i = 0; i < UE_num; i++) {
        double required_data_rate = uniform(generator);
        // double required_data_rate = normal(generator);
        // double required_data_rate = 0;
        // input >> required_data_rate;

        Ptr<MobilityModel> UE_mobility_model = (UE_nodes.Get(i))->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        // my_UE_list.push_back(MyUeNode(i, pos, required_data_rate));
        my_UE_list.push_back(MyUeNode(i, pos, required_data_rate));
    }

    return my_UE_list;
}
