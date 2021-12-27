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
double getIrradianceAngle(Ptr<Node> AP, Ptr<Node> UE) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();

    Ptr<MobilityModel> UE_mobility_model = UE->GetObject<MobilityModel>();
    Vector UE_pos = UE_mobility_model->GetPosition();

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;

    const plane_dist = sqrt(dx*dx + dy*dy);
    const double height_diff = AP_pos.z - UE_pos.z;

    return atan(plane_dist / height_diff);
}

// cosψ = a*sin(θ) + b*cos(θ)
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP_node, Ptr<Node> UE_node, MyUeNode &UE) {
    double theta = getRandomOrientation(UE);

    Ptr<MobilityModel> VLC_AP_mobility = VLC_AP_node->GetObject<MobilityModel>();
    Vector AP_pos = VLC_AP_mobility->GetPosition();

    Ptr<MobilityModel> UE_mobility = UE_node->GetObject<MobilityModel>();
    Vector UE_curr_pos = UE_mobility->GetPosition();
    Vector UE_next_pos = UE_mobility->GetNextPosition();
    UE.setPosition(UE_curr_pos);

    double Omega = atan((UE_next_pos.y - UE_curr_pos.y) / (UE_next_pos.x - UE_curr_pos.x));

    double AP_UE_dx = AP_pos.x - UE_curr_pos.x;
    double AP_UE_dy = AP_pos.y - UE_curr_pos.y;
    double AP_UE_dz = AP_pos.z - UE_curr_pos.z;
    double d = sqrt(AP_UE_dx*AP_UE_dx + AP_UE_dy*AP_UE_dy + AP_UE_dz*AP_UE_dz);

    double a = -1 * (AP_UE_dx/d) * cos(Omega) - (AP_UE_dy/d) * sin(Omega);
    double b = (AP_UE_dz) / d;

    return a * sin(theta) + b * cos(theta);
}

// θ
double getRandomOrientation(MyUeNode &UE) {
    UE.randomAnOrientationAngle();
    return UE.getOrientationAngle();
}


/*
    VLC channel gain, including LOS and front-end
*/
// line of sight
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE_node, MyUeNode &UE) {
    const double irradiance_angle = getIrradianceAngle(VLC_AP, UE_node); // the irradiance angle of the Tx (Φ)

    const double cosine_incidence_angle = getCosineOfIncidenceAngle(VLC_AP, UE_node, UE); // cos(ψ)
    if (radian2Degree(acos(cosine_incidence_angle)) > field_of_view / 2)
        return 0.0;


    const double concentrator_gain = pow(refractive_index, 2) / pow(sin(degree2Radian(field_of_view/2)), 2);
    const double lambertian_coefficient = (-1) / (log(cos(degree2Radian(PHI_half))));
    const double distance = getDistance(VLC_AP, UE);

    double line_of_sight = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    line_of_sight = line_of_sight * concentrator_gain;
    line_of_sight = line_of_sight * filter_gain;
    line_of_sight = line_of_sight * pow(cos(irradiance_angle), lambertian_coefficient);
    line_of_sight = line_of_sight * cosine_incidence_angle;

    return line_of_sight;
}

double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes,std::vector<MyUeNode> &myUElist, std::vector<std::vector<double>> &VLC_LOS_matrix) {
    for (int i = 0; i < VLC_AP_num; i++)
	{
		for (int j = 0; j < UE_num; j++)
		{
			VLC_LOS_matrix[i][j] = estimateOneVlcLightOfSight(VLC_AP_nodes.Get(i), UE_nodes.Get(j), myUElist(j));
		}
	}
}

// front-end
// H_F(k) = exp( -(k * modulation_bandwidth) / (subcarrier_num*fitting_coefficient*3dB_cutoff))
double estimateOneVlcFrontEnd(int subcarrier_index) {
    return exp((-1) * subcarrier_index * VLC_AP_bandwidth / (subcarrier_num * fitting_coefficient * three_dB_cutoff));
}


/*
    VLC SINR
*/
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double interference = 0;
    for (int i = 0; i < VLC_AP_num; i++) {
        if (i != VLC_AP_index)
            interference += pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2);
    }

    double noise = pow(optical_to_electric_power_ratio, 2) * VLC_AP_bandwidth * noise_power_spectral_density;
    double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_channel_gain_matrix[VLC_AP_index][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2) / (interference+noise);

    return SINR;
}

void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix) {
    for (int i = 0; i < VLC_AP_Num; i++) {
		for (int j = 0; j < UE_Num; j++) {
			for (int k = 0; k < subcarrier_num; k++) {
                VLC_SINR_matrix[i][j][k] = estimateOneVlcSINR(VLC_LOS_matrix, i, j, k);
			}
		}
	}
}


/*
    RF data rate

    NOTE:
    - Slot time is not given in the benchmark.
    - RTS/CTS is much shorter than SIFS, PIFS, and DIFS in the benchmark.
    However, the situation is opposite in "Downlink and uplink resource allocation in IEEE 802.11 wireless LANs".
*/

double calculateRfSystemUtilization(int serving_num) {
    double t_c = RTS_time + DIFS_time;
    double t_s = RTS_time + CTS_time + header_time + propagation_delay + ACK_time + 3*SIFS_time + DIFS_time;
    double t_d = header_time + propagation_delay + ACK_time + SIFS_time + PIFS_time;

    double p_c = 2 / (max_backoff_stage+1);
    double p_t = 1 - pow(1-p_c, serving_num+1);
    double p_s = ((serving_num+1) * p_c * pow(1-p_c, serving_num)) / (p_t);
    double p_d = (serving_num - 1) / (2 * serving_num * p_s);

    double denominator = (1 - p_t) * slot_time;
    denominator += p_t * p_s * (1 - p_d) * t_s;
    denominator += p_t * p_s * p_d * t_d;
    denominator += p_t * (1 - p_s) * t_c;

    return (p_s * p_t * propagation_delay) / (denominator);
}

double calculateRfDownlinkUtilizationEfficiency(int serving_num) {
    double system_utilization = calculateSystemUtilization(serving_num);

    return system_utilization * utilization_ratio / (1+utilization_ratio);
}

// can be calculated prior to the simulation
// pre-calculate RF downlink utilization efficiency for all possible number of serving UEs
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

double estimateOneVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double numerator = 2 * VLC_AP_bandwidth * getSpectralEfficiency(VLC_SINR_matrix[VLC_AP_index][UE_index][subcarrier_index]);
    double denominator = subcarrier_num * time_slot_num;

    return numerator / denominator;
}


void calculateAllVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix) {
    for (int i = 0; i < VLC_AP_Num; i++) {
		for (int j = 0; j < UE_Num; j++) {
			for (int k = 0; k < subcarrier_num; k++) {
                VLC_data_rate_matrix[i][j][k] = estimateOneVlcDataRate(VLC_SINR_matrix, i, j, k);
			}
		}
	}

}



