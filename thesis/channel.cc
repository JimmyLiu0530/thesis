#include <cmath>


#include "channel.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


/*
    distance and angle calculation
*/

double radian2Degree(const double &radian) {
    return radian * 180 / PI;
}

double degree2Radian(const double &degree) {
    return degree * PI / 180;
}

double getDistance(Ptr<Node> AP, MyUeNode &UE) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();
    Vector UE_pos = UE.pos;

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;
    double dz = AP_pos.z - UE_pos.z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}

/*
       plane_dist
    AP----------
    |Φ\        |
    |  \       |
    |   \      |
    |    \     | height_diff
    |     \    |
    |      \   |
    |       \  |
    |        \ |
              UE (PD)

    arctan(plane_dist / height_diff) = Φ
*/
double getIrradianceAngle(Ptr<Node> AP, MyUeNode &UE) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();
    Vector UE_pos = UE.pos;

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;

    const plane_dist = sqrt(dx*dx + dy*dy);
    const double height_diff = AP_pos.z - UE_pos.z;

    return atan(plane_dist / height_diff);
}


double randomOrientation(MyUeNode &UE) {
    UE.randomAnOrientationAngle();
    return UE.getOrientationAngle();
}


/*
    VLC channel gain, including front-end and LOS
*/

// H_F(k) = exp( -(k * modulation_bandwidth) / (subcarrier_num*fitting_coefficient*3dB_cutoff))
double estimateOneVlcFrontEnd(int subcarrier_index) {

    return exp((-1) * subcarrier_index * VLC_AP_bandwidth / (subcarrier_num * fitting_coefficient * three_dB_cutoff));
}

double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, MyUeNode &UE) {
    const double irradiance_angle = getIrradianceAngle(VLC_AP, UE); // the irradiance angle of the Tx (Φ)

    const double incidence_angle = irradiance_angle + randomOrientation(UE);// the incidence angle of the receiving PD (ψ) = irradiance_angle (Φ) + θ
    if (radian2Degree(incidence_angle) >= field_of_view / 2)
        return 0.0;


    const double concentrator_gain = pow(refractive_index, 2) / pow(sin(degree2Radian(field_of_view/2)), 2);
    const double lambertian_coefficient = (-1) / (log(cos(degree2Radian(PHI_half))));
    const double distance = getDistance(VLC_AP, UE);

    double line_of_sight = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    line_of_sight = line_of_sight * concentrator_gain;
    line_of_sight = line_of_sight * filter_gain;
    line_of_sight = line_of_sight * pow(cos(irradiance_angle), lambertian_coefficient);
    line_of_sight = line_of_sight * cos(incidence_angle);

    return line_of_sight;
}

double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, std::vector<MyUeNode> &myUElist, std::vector<std::vector<double>> &VLC_LOS_matrix) {
    for (int i = 0; i < VLC_AP_num; i++)
	{
		for (int j = 0; j < myUElist.size(); j++)
		{
			VLC_LOS_matrix[i][j] = estimateOneVlcLightOfSight(VLC_AP_nodes.Get(i), myUElist(j));
		}
	}
}


/*
    VLC SINR
*/

double calculateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double interference = 0;
    for (int i = 0; i < VLC_AP_num; i++) {
        if (i != VLC_AP_index)
            interference += pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2);
    }

    double noise = pow(optical_to_electric_power_ratio, 2) * VLC_AP_bandwidth * noise_power_spectral_density;
    double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_channel_gain_matrix[VLC_AP_index][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2) / (interference+noise);

    return SINR;
}


/*
    RF data rate

    NOTE:
    - Slot time is not given in the benchmark.
    - RTS/CTS is much shorter than SIFS, PIFS, and DIFS in the benchmark.
    However, the situation is opposite in "Downlink and uplink resource allocation in IEEE 802.11 wireless LANs".
*/

double calculateSystemUtilization(int num) {
    double t_c = RTS_time + DIFS_time;
    double t_s = RTS_time + CTS_time + header_time + propagation_delay + ACK_time + 3*SIFS_time + DIFS_time;
    double t_d = header_time + propagation_delay + ACK_time + SIFS_time + PIFS_time;

    double p_c = 2 / (max_backoff_stage+1);
    double p_t = 1 - pow(1-p_c, num+1);
    double p_s = ((num+1) * p_c * pow(1-p_c, num)) / (p_t);
    double p_d = (num - 1) / (2 * num * p_s);

    double denominator = (1 - p_t) * slot_time;
    denominator += p_t * p_s * (1 - p_d) * t_s;
    denominator += p_t * p_s * p_d * t_d;
    denominator += p_t * (1 - p_s) * t_c;

    return (p_s * p_t * propagation_delay) / (denominator);
}

double calculateDownlinkUtilizationEfficiency(int num) {
    double system_utilization = calculateSystemUtilization(num);

    return system_utilization * utilization_ratio / (1+utilization_ratio);
}

// can be calculated prior to the simulation
void calculateAllRfDownlinkUtilizationEfficiency(std::vector<double> &downlink_utilization_efficiency) {
    for (int i = 1; i <= UE_num; i++) {
        downlink_utilization_efficiency.push_back(calculateDownlinkUtilizationEfficiency(i));
    }
}


/*
    VLC data rate
*/

double getSpectralEfficiency(double SINR) {
    std::map<char,int>::iterator it = SINR_to_spectral_efficiency.lower_bound(SINR);

    return it->second;
}

void calculateOneVlcDataRate(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double numerator = 2 * VLC_AP_bandwidth * getSpectralEfficiency(calculateOneVlcSINR(VLC_LOS_matrix, VLC_AP_index, UE_index, subcarrier_index));
    double denominator = subcarrier_num * time_slot_num;

    return numerator / denominator;
}




