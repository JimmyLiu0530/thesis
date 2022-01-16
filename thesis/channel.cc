#include <cmath>


#include "channel.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"



/*
    table of conversion from SINR to spectral efficiency
*/
const std::map<double, double, std::greater<double>> SINR_to_spectral_efficiency = { {1.0, 0.877}, {3.0, 1.1758}, {5.0, 1.4766},
                                                                                     {8.0, 1.9141}, {9.0, 2.4063}, {11.0, 2.7305},
                                                                                     {12.0, 3.3223}, {14.0, 3.9023}, {16.0, 4.5234},
                                                                                     {18.0, 5.1152}, {20.0, 5.5547} };



static int counter = 0;


void precalculation(NodeContainer  &RF_AP_node,
                      NodeContainer  &VLC_AP_nodes,
                      NodeContainer  &UE_nodes,
                      std::vector<std::vector<double>> &VLC_LOS_matrix,
                      std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix,
                      std::vector<double> &RF_data_rate_vector,
                      std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                      std::vector<MyUeNode> &my_UE_list)
{
    calculateAllVlcLightOfSight(VLC_AP_nodes, UE_nodes, my_UE_list, VLC_LOS_matrix);

#if DEBUG_MODE
    print_VLC_LOS_Matrix(VLC_Channel_Gain_Matrix);
#endif

    calculateAllVlcSINR(VLC_LOS_matrix, VLC_SINR_matrix);

#if DEBUG_MODE
    print_VLC_SINR_Matrix(VLC_SINR_Matrix);
#endif

    // data rate for RF
    //
    // since RF data rate only depends on the number of serving UE,
    // here we pre-calculate all possible data rates under different number of serving UEs,
    // thus we do this once and for all
    if (!counter) {
        counter++;
        calculateRfDataRate(RF_data_rate_vector);
    }
    // data rate for VLC
    calculateAllVlcDataRate(VLC_SINR_matrix, VLC_data_rate_matrix);

#if DEBUG_MODE
    print_RF_DataRate_Matrix(RF_DataRate_Matrix);
    print_VLC_DataRate_Matrix(VLC_DataRate_Matrix);
  #endif
}



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
    Vector UE_pos = UE.getPosition();

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

    const double plane_dist = sqrt(dx*dx + dy*dy);
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
    UE.setPosition(UE_curr_pos);

    Ptr<RandomWaypointMobilityModel> rand_UE_mobility = StaticCast<RandomWaypointMobilityModel, MobilityModel> (UE_mobility);
    Vector UE_next_pos = rand_UE_mobility->GetNextPosition();

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
			VLC_LOS_matrix[i][j] = estimateOneVlcLightOfSight(VLC_AP_nodes.Get(i), UE_nodes.Get(j), myUElist[j]);
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
    double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[VLC_AP_index][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2) / (interference+noise);

    return SINR;
}

void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix) {
    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			for (int k = 0; k < effective_subcarrier_num; k++) {
                VLC_SINR_matrix[i][j][k] = estimateOneVlcSINR(VLC_LOS_matrix, i, j, k);
			}
		}
	}
}


/*
    RF data rate for each UE connected to WiFi

    NOTE:
    - Slot time is not given in the benchmark.
    - RTS/CTS is much shorter than SIFS, PIFS, and DIFS in the benchmark.
    However, the situation is opposite in "Downlink and uplink resource allocation in IEEE 802.11 wireless LANs".
*/

double calculateRfSystemUtilization(int serving_UE_num) {
    double t_c = RTS_time + DIFS_time;
    double t_s = RTS_time + CTS_time + header_time + propagation_delay + ACK_time + 3*SIFS_time + DIFS_time;
    double t_d = header_time + propagation_delay + ACK_time + SIFS_time + PIFS_time;

    double p_c = 2 / (max_backoff_stage+1);
    double p_t = 1 - pow(1-p_c, serving_UE_num+1);
    double p_s = ((serving_UE_num+1) * p_c * pow(1-p_c, serving_UE_num)) / (p_t);
    double p_d = (serving_UE_num - 1) / (2 * serving_UE_num * p_s);

    double denominator = (1 - p_t) * slot_time;
    denominator += p_t * p_s * (1 - p_d) * t_s;
    denominator += p_t * p_s * p_d * t_d;
    denominator += p_t * (1 - p_s) * t_c;

    return (p_s * p_t * propagation_delay) / (denominator);
}

double calculateRfDownlinkUtilizationEfficiency(int serving_UE_num) {
    double system_utilization = calculateRfSystemUtilization(serving_UE_num);

    return system_utilization * utilization_ratio / (1+utilization_ratio);
}

void calculateRfDataRate(std::vector<double> &RF_data_rate_vector) {
    // for the case when no serving UE
    RF_data_rate_vector.push_back(0);

    for (int serving_UE_num = 1; serving_UE_num <= UE_num; serving_UE_num++) {
        double downlink_utilization_eff = calculateRfDownlinkUtilizationEfficiency(serving_UE_num);
        RF_data_rate_vector.push_back(channel_bit_rate * downlink_utilization_eff / serving_UE_num);
    }
}


/*
    VLC data rate
*/

// return the corresponding spectral efficiency of the given SINR according to the pre-established table
double getSpectralEfficiency(double SINR) {
    auto it = SINR_to_spectral_efficiency.lower_bound(SINR);

    return it->second;
}

// data rate of the RU on the certain subcarrier based on (6)
double estimateOneVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double numerator = 2 * VLC_AP_bandwidth * getSpectralEfficiency(VLC_SINR_matrix[VLC_AP_index][UE_index][subcarrier_index]);
    double denominator = subcarrier_num * time_slot_num;

    return numerator / denominator;
}


void calculateAllVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix) {
    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			for (int k = 0; k < effective_subcarrier_num; k++) {
                VLC_data_rate_matrix[i][j][k] = estimateOneVlcDataRate(VLC_SINR_matrix, i, j, k);
			}
		}
	}

}



