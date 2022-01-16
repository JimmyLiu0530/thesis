#ifndef MY_UE_NODE_LIST_H
#define MY_UE_NODE_LIST_H

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

#include "global_configuration.h"
#include "my_UE_node.h"

using namespace ns3;

std::vector<MyUeNode> initializeMyUeNodeList(NodeContainer &UE_nodes);

#endif // MY_UE_NODE_LIST_H
