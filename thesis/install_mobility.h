#ifndef INSTALL_MOBILITY_H
#define INSTALL_MOBILITY_H

using namespace ns3;

void installRfApMobility(NodeContainer &RF_AP_nodes);
void installVlcApMobility(NodeContainer &VLC_AP_nodes);
void installUeMobility(NodeContainer &UE_nodes);



#endif // INSTALL_MOBILITY_H
