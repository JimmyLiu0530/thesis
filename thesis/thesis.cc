/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//
// Network topology
//
//           10Mb/s, 10ms       10Mb/s, 10ms
//       n0-----------------n1-----------------n2
//
//
// - Tracing of queues and packet receptions to file
//   "tcp-large-transfer.tr"
// - pcap traces also generated in the following files
//   "tcp-large-transfer-$n-$i.pcap" where n and i represent node and interface
// numbers respectively
//  Usage (e.g.): ./waf --run tcp-large-transfer

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <iomanip>
#include <string>

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "global_configuration.h"
#include "install_mobility.h"
#include "my_UE_node.h"
#include "my_UE_node_list.h"
#include "channel.h"
#include "print.h"
#include "benchmark.h"
#include "proposed_method.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TcpLargeTransfer");
std::vector<double> Received(1, 0.0);
std::vector<double> theTime(1, 0.0);

std::vector<double> discount_ratio_per_AP(RF_AP_num + VLC_AP_num, 0.8);
// std::vector<double> avg_satisfaction_per_AP(RF_AP_num + VLC_AP_num, 0.0);
std::vector<std::vector<int>> AP_association_matrix(RF_AP_num + VLC_AP_num, std::vector<int> (UE_num, 0));


std::vector<int> RF_cnt; // the number of UEs that the RF AP serves in each state
std::vector<double> RF_data_rate_vector(UE_num+1, 0.0); // in Mbps

std::vector<std::vector<double>> VLC_LOS_matrix(VLC_AP_num, std::vector<double> (UE_num, 0.0));
std::vector<std::pair<int, int>> first_empty_RU_position (VLC_AP_num, std::make_pair(effective_subcarrier_num, time_slot_num - 1)); // the position of the first empty RU for each VLC AP (view from high freq to low)
std::vector<std::vector<std::vector<double>>> VLC_SINR_matrix(VLC_AP_num, std::vector<std::vector<double>> (UE_num, std::vector<double> (subcarrier_num, 0.0))); // in dB
std::vector<std::vector<std::vector<double>>> VLC_data_rate_matrix(VLC_AP_num, std::vector<std::vector<double>> (UE_num, std::vector<double> (subcarrier_num, 0.0))); // in Mbps
std::vector<std::vector<std::vector<int>>> RU_matrix_per_VLC_AP(VLC_AP_num, std::vector<std::vector<int>> (effective_subcarrier_num+1, std::vector<int> (time_slot_num, 0)));



std::vector<double> recorded_avg_throughput_per_state(state_num, 0.0); // in Mbps
std::vector<double> recorded_avg_satisfaction_per_state(state_num, 0.0);
std::vector<double> recorded_satisfaction_fairness_per_state(state_num, 0.0);
std::vector<double> recorded_throughput_fairness_per_state(state_num, 0.0);



static const uint32_t totalTxBytes = 10000000;
static uint32_t currentTxBytes = 0;
static const uint32_t writeSize = 1040;
static int state = 0;
uint8_t data[writeSize];
std::string path = "/home/hsnl/repos/ns-3-allinone/ns-3.25/scratch/thesis/output.csv";


void StartFlow(Ptr<Socket>, Ipv4Address, uint16_t);
void WriteUntilBufferFull(Ptr<Socket>, uint32_t);

std::string intToString(const int& num){
	std::stringstream ss;
	ss << num;
	return ss.str();
}

/*
  used for tracing and calculating throughput
 */
static void RxEndAddress(Ptr<const Packet> p, const Address &address) {
    // appends on the received packet to the received data up until that packet
    // and adds that total to the end of the vector
    Received.push_back(Received.back() + p->GetSize());
    // total += p->GetSize();
    theTime.push_back(Simulator::Now().GetSeconds()); // keeps track of time during simulation that a packet is received
    double throughput = ((Received.back() * 8)) / theTime.back(); // goodput calculation
    std::cout << "Received.back() is :" << Received.back() << std::endl;
    std::cout << "Rx throughput value is :" << throughput << std::endl;
    std::cout << "Current time is :" << theTime.back() << std::endl;
    std::cout << "Received size: " << p->GetSize() << " at: " << Simulator::Now().GetSeconds() << "s"
              << "IP: " << InetSocketAddress::ConvertFrom(address).GetIpv4() << std::endl;
}


void updateToNextState(NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<MyUeNode> &my_UE_list)
{
#if DEBUG_MODE
    std::cout << "updateToNextState(" << state << ")\n";
#endif


#if PROPOSED_METHOD

    proposedDynamicLB(state, RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix, RF_data_rate_vector,
                      VLC_data_rate_matrix, AP_association_matrix, RU_matrix_per_VLC_AP,
                      discount_ratio_per_AP, first_empty_RU_position, my_UE_list);

#else

    benchmarkDynamicLB(state, RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix,
                        VLC_SINR_matrix, RF_data_rate_vector, VLC_data_rate_matrix,
                        AP_association_matrix, my_UE_list);

#endif

    // since we would adjust the order of my_UE_list during APA&RA, we have to reorder it back after the end of each state
    std::sort(my_UE_list.begin(), my_UE_list.end(), [](MyUeNode a, MyUeNode b){return a.getID() < b.getID();});


    // 1. calculate the number of UE connected to the RF AP
    int cnt = 0;
    for (int i = 0; i < AP_association_matrix[0].size(); i++)
        if (AP_association_matrix[0][i] == 1)
            cnt++;

    RF_cnt.push_back(cnt);

    // 2. calculate the avg data rate and satisfaction of the current state
    double avg_data_rate = 0.0;
    double avg_satisfaction = 0.0;

    for (int i = 0; i < my_UE_list.size(); i++) {
        avg_data_rate += my_UE_list[i].getLastThroughput();
        avg_satisfaction += my_UE_list[i].getLastSatisfaction();
    }
    avg_data_rate = avg_data_rate / UE_num;
    avg_satisfaction = avg_satisfaction / UE_num;


    // 3. calculate the Jain's fairness of satisfaction
    double satisfaction_fairness = 0.0;
    double square_of_sum = 0.0;
    double sum_of_square = 0.0;

    for (int i = 0; i < UE_num; i++) {
        double satisfaction = my_UE_list[i].getLastSatisfaction();

        square_of_sum += satisfaction;
        sum_of_square += pow(satisfaction, 2);
    }
    square_of_sum = pow(square_of_sum, 2);
    satisfaction_fairness = square_of_sum / (UE_num * sum_of_square);


    // 4. calculate the fairness of throughput
    double throughput_fairness = 0.0;
    square_of_sum = 0.0;
    sum_of_square = 0.0;

    for (int i = 0; i < UE_num; i++) {
        double throughput = my_UE_list[i].getLastThroughput();

        square_of_sum += throughput;
        sum_of_square += pow(throughput, 2);
    }
    square_of_sum = pow(square_of_sum, 2);
    throughput_fairness = square_of_sum / (UE_num * sum_of_square);


#if DEBUG_MODE
    std::cout << "state " << state << std::endl;
    std::cout << "avg data rate: " << avg_data_rate << ", ";
    std::cout << "avg satisfaction: " << avg_satisfaction << ", ";
    std::cout << "satisfaction fairness: " << satisfaction_fairness << ", ";
    std::cout << "throughput fairness: " << throughput_fairness << std::endl;
    std::cout << "RF connection ratio: " << (double)cnt / UE_num * 100 << "%" << std::endl << std::endl;
#endif // DEBUG_MODE


    // use another storage to keep some information of each UE
    // since somehow get 0 when accessing these information through my_UE_list after Simulator::Run()
    recorded_avg_throughput_per_state[state] = avg_data_rate;
    recorded_avg_satisfaction_per_state[state] = avg_satisfaction;
    recorded_satisfaction_fairness_per_state[state] = satisfaction_fairness;
    recorded_throughput_fairness_per_state[state] = throughput_fairness;

    state++;

    if (!Simulator::IsFinished())
        Simulator::Schedule(Seconds(time_period), &updateToNextState, RF_AP_node, VLC_AP_nodes, UE_nodes, my_UE_list);

}


int main(int argc, char *argv[])
{
    // Users may find it convenient to turn on explicit debugging
    // for selected modules; the below lines suggest how to do this
    //  LogComponentEnable("TcpL4Protocol", LOG_LEVEL_ALL);
    //  LogComponentEnable("TcpSocketImpl", LOG_LEVEL_ALL);
    //  LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
    //  LogComponentEnable("TcpLargeTransfer", LOG_LEVEL_ALL);

    CommandLine cmd;
    cmd.Parse (argc, argv);

    Config::SetDefault("ns3::TcpSocket::SegmentSize",UintegerValue(1000));
    Config::SetDefault("ns3::TcpSocket::SndBufSize",UintegerValue(1000000000));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize",UintegerValue(1000000000));

    // initialize the tx buffer.
    for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }


    // create RF AP node
    NodeContainer RF_AP_node;
    RF_AP_node.Create(RF_AP_num);
    installRfApMobility(RF_AP_node);

#if DEBUG_MODE
    printRfApPosition(RF_AP_node);
#endif

    // create VLC AP nodes
    NodeContainer VLC_AP_nodes;
    VLC_AP_nodes.Create (VLC_AP_num);
    installVlcApMobility(VLC_AP_nodes);

#if DEBUG_MODE
    printVlcApPosition(VLC_AP_nodes);
#endif

    // create UE nodes
    NodeContainer UE_nodes;
    UE_nodes.Create (UE_num);
    installUeMobility(UE_nodes);

#if DEBUG_MODE
    printUePosition(UE_nodes);
#endif

    std::vector<MyUeNode> my_UE_list = initializeMyUeNodeList(UE_nodes);

#if DEBUG_MODE
    // printMyUeList(my_UE_list);
#endif

    /** add ip/tcp stack to all nodes.**/
    // InternetStackHelper internet;
    // internet.InstallAll ();




    // /** set p2p helper **/
    // PointToPointHelper p2p;
    // p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
    // p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (0.1)));



    // /*
    //   Channel[i][j] 放的是 AP i - UE j的link
    //   該link用Netdevicecontainer表達
    //   Netdevicecontainer.Get(0) = transmitter device
    //   Netdevicecontainer.Get(1) = receiver device
    // */

    // std::vector<std::vector<NetDeviceContainer>> VLC_Channel(VLC_AP_Num,std::vector<NetDeviceContainer> (UE_Num));
    // for(int i=0;i<VLC_AP_Num;i++){
    //   for(int j=0;j<UE_Num;j++){
    //     VLC_Channel[i][j]=p2p.Install(VLC_AP_Nodes.Get(i),UE_Nodes.Get(j));
    //   }
    // }

    // /** Later, we add IP addresses. **/
    // std::vector<Ipv4InterfaceContainer> ipvec;
    // Ipv4AddressHelper ipv4;
    // for(int i=0;i<VLC_AP_Num;i++){
    //   for(int j=0;j<UE_Num;j++){
    //     std::string addressStr = "10."+intToString(i+1) +"." + intToString(j+1) + ".0";
    //     ipv4.SetBase(addressStr.c_str(),"255.255.255.0");
    //     ipvec.push_back(ipv4.Assign (VLC_Channel[i][j]));

    //     #if DEBUG_MODE
    //       std::cout<<ipvec.back().GetAddress(1)<<" ";
    //     #endif
    //   }
    //   #if DEBUG_MODE
    //     std::cout << std::endl;
    //   #endif
    // }



    // // set serve Port
    // uint16_t servPort = 50000;

    // // Create a packet sink to receive these packets on n2...
    // PacketSinkHelper sink ("ns3::TcpSocketFactory",
    //                        InetSocketAddress (Ipv4Address::GetAny (), servPort));

    // ApplicationContainer apps ;
    // apps.Add(sink.Install(UE_Nodes));
    // apps.Start (Seconds (0.0));
    // apps.Stop (Seconds (6.0));



    // // Create and bind the socket...
    // std::vector<std::vector<Ptr<Socket>>> localSockets(VLC_AP_Num,std::vector<Ptr<Socket> > (UE_Num));
    // for(int i=0;i<VLC_AP_Num;i++){
    //   for(int j=0;j<UE_Num;j++){
    //     localSockets[i][j] = Socket::CreateSocket (VLC_AP_Nodes.Get (i), TcpSocketFactory::GetTypeId ());
    //     localSockets[i][j]->Bind();
    //   }
    // }
    // // Trace changes to the congestion window
    // // Config::ConnectWithoutContext ("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));

    // ApplicationContainer::Iterator i;
    // for (i = apps.Begin (); i != apps.End (); ++i){
    //         (*i)->TraceConnectWithoutContext("Rx", MakeCallback(&RxEndAddress));
    // }
    // // ...and schedule the sending "Application"; This is similar to what an
    // // ns3::Application subclass would do internally.

    // int ipvec_index=0;
    // for(int i=0;i<VLC_AP_Num;i++){
    //   for(int j=0;j<UE_Num;j++){
    //     Simulator::Schedule(Seconds(0.0),&StartFlow,localSockets[i][j],ipvec[ipvec_index++].GetAddress(1), servPort);
    //   }
    // }


    Simulator::Schedule(Seconds(0.0), &updateToNextState, RF_AP_node, VLC_AP_nodes, UE_nodes, my_UE_list);

    Simulator::Stop(Seconds(state_num * time_period)); // 1000 quasi-static states in total
    Simulator::Run();




    /*
     * after simulation, calculate overall throughput, fairness index and satisfaction
    */

    // overall avg. throughput, satisfaction, satisfaction variance, and fairness index
    double avg_throughput = 0.0;
    double avg_satisfaction = 0.0;
    double avg_satisfaction_fairness = 0.0;
    double avg_throughput_fairness = 0.0;

    for (int i = 0; i < state_num; i++) {
        avg_throughput += recorded_avg_throughput_per_state[i];
        avg_satisfaction += recorded_avg_satisfaction_per_state[i];
        avg_satisfaction_fairness += recorded_satisfaction_fairness_per_state[i];
        avg_throughput_fairness += recorded_throughput_fairness_per_state[i];
    }
    avg_throughput = avg_throughput / state_num;
    avg_satisfaction = avg_satisfaction / state_num;
    avg_satisfaction_fairness = avg_satisfaction_fairness / state_num;
    avg_throughput_fairness = avg_throughput_fairness / state_num;


    // average percentage of WiFi connections
    double avg_RF_connection_ratio = 0.0;
    for (int i = 0; i < RF_cnt.size(); i++)
        avg_RF_connection_ratio += ((double)RF_cnt[i] / UE_num);

    avg_RF_connection_ratio /= RF_cnt.size();


    std::cout << "In this experiment, ";
    std::cout << "avg. throughput: " << avg_throughput << " Mbps, ";
    std::cout << "avg. satisfaction: " << avg_satisfaction << ", ";
    std::cout << "satisfaction fairness: " << avg_satisfaction_fairness << ", ";
    std::cout << "throughput fairness: " << avg_throughput_fairness << std::endl << std::endl;
    //::cout << "RF connection percentage: " << avg_RF_connection_ratio*100 << "%" << std::endl;



    /*
     * output the results to .csv files
    */
    std::fstream output;
    output.open(path, std::ios::out | std::ios::app);
    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
    }
    else {
        output << avg_throughput << "," << avg_satisfaction << "," << avg_satisfaction_fairness << "," << avg_throughput_fairness << ",";// << avg_RF_connection_ratio << ",";
        output << std::endl;
    }


    output.close();
    Simulator::Destroy();
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//begin implementation of sending "Application"
void StartFlow (Ptr<Socket> localSocket,
                Ipv4Address servAddress,
                uint16_t servPort)
{
  NS_LOG_LOGIC ("Starting flow at time " <<  Simulator::Now ().GetSeconds ());
  localSocket->Connect (InetSocketAddress (servAddress, servPort)); //connect

  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available
  //localSocket->SetSendCallback (MakeCallback (&WriteUntilBufferFull));
  WriteUntilBufferFull (localSocket, localSocket->GetTxAvailable ());
  currentTxBytes=0;
}

void WriteUntilBufferFull (Ptr<Socket> localSocket, uint32_t txSpace)
{
  while (currentTxBytes < totalTxBytes && localSocket->GetTxAvailable () > 0)
    {
      uint32_t left = totalTxBytes - currentTxBytes;
      uint32_t dataOffset = currentTxBytes % writeSize;
      uint32_t toWrite = writeSize - dataOffset;
      toWrite = std::min (toWrite, left);
      toWrite = std::min (toWrite, localSocket->GetTxAvailable ());
      int amountSent = localSocket->Send (&data[dataOffset], toWrite, 0);
      if(amountSent < 0)
        {
          // we will be called again when new tx space becomes available.
          return;
        }
      currentTxBytes += amountSent;
    }
  localSocket->Close ();
}
