#include <cmath>
#include <iostream>
#include <iomanip>

#include "channel.h"
#include "print.h"
#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"



static int counter = 0;
static const double lambertian_coefficient = (-1) / (log2(cos(degree2Radian(PHI_half)))); // m
static const double concentrator_gain = pow(refractive_index, 2) / pow(sin(degree2Radian(field_of_view/2)), 2);


/*
    table of conversion from SINR to spectral efficiency

    bit/s/Hz = 1 Mbit/s/MHz
*/
const std::map<double, double, std::greater<double>> SINR_to_spectral_efficiency = { {0.0, 0}, {1.0, 0.877}, {3.0, 1.1758}, {5.0, 1.4766},
                                                                                     {8.0, 1.9141}, {9.0, 2.4063}, {11.0, 2.7305}, {12.0, 3.3223},
                                                                                     {14.0, 3.9023}, {16.0, 4.5234}, {18.0, 5.1152}, {20.0, 5.5547} };


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
    printVlcLosMatrix(VLC_LOS_matrix);
#endif

    calculateAllVlcSINR(VLC_LOS_matrix, VLC_SINR_matrix);

#if DEBUG_MODE
    printVlcSinrMatrix(VLC_SINR_matrix);
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
    printRfDataRateVector(RF_data_rate_vector);
    printVlcDataRateMatrix(VLC_data_rate_matrix);
  #endif
}

double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes,std::vector<MyUeNode> &my_UE_list, std::vector<std::vector<double>> &VLC_LOS_matrix) {
    for (int i = 0; i < UE_num; i++)
        my_UE_list[i].randomOrientationAngle(UE_nodes.Get(i));

    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			VLC_LOS_matrix[i][j] = estimateOneVlcLightOfSight(VLC_AP_nodes.Get(i), UE_nodes.Get(j), my_UE_list[j]);
		}
	}
}

/*
    VLC channel gain, including LOS and front-end
*/
// line of sight
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node) {
    const double cosine_incidence_angle = getCosineOfIncidenceAngle(VLC_AP, UE, UE_node); // cos(ψ)
    if (radian2Degree(acos(cosine_incidence_angle)) > field_of_view / 2) // incidence angle exceeds half of FoV
        return 0.0;

    const double irradiance_angle = getIrradianceAngle(VLC_AP, UE_node); // the irradiance angle(Φ) of the Tx (in rad)
    const double distance = getDistance(VLC_AP, UE_node);

    double line_of_sight = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    line_of_sight = line_of_sight * concentrator_gain;
    line_of_sight = line_of_sight * filter_gain;
    line_of_sight = line_of_sight * pow(cos(irradiance_angle), lambertian_coefficient);
    line_of_sight = line_of_sight * cosine_incidence_angle;

    return line_of_sight;
}

// cosψ = 1/d((x_a-x_u)sinθcosω+(y_a-y_u)sinθsinω+(z_a-z_u)cosθ)
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node) {
    double polar_angle = UE_node.getPolarAngle();
    double azimuth_angle = UE_node.getAzimuthAngle();

    Ptr<MobilityModel> VLC_AP_mobility = VLC_AP->GetObject<MobilityModel>();
    Vector AP_pos = VLC_AP_mobility->GetPosition();

    Ptr<MobilityModel> UE_mobility = UE->GetObject<MobilityModel>();
    Vector UE_curr_pos = UE_mobility->GetPosition();
    UE_node.setPosition(UE_curr_pos);

    double dx = AP_pos.x - UE_curr_pos.x;
    double dy = AP_pos.y - UE_curr_pos.y;
    double dz = AP_pos.z - UE_curr_pos.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);

    double first_term = dx * sin(polar_angle) * cos(azimuth_angle);
    double second_term = dy * sin(polar_angle) * sin(azimuth_angle);
    double last_term = dz * cos(polar_angle);

    return (first_term + second_term + last_term) / dist;
}

/*
    distance and angle calculation

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

    arctan(plane_dist / height_diff) = Φ (rad)
*/
double getIrradianceAngle(Ptr<Node> AP, MyUeNode &UE_node) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();

    Vector UE_pos = UE_node.getPosition();

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;

    const double plane_dist = sqrt(dx*dx + dy*dy);
    const double height_diff = AP_pos.z - UE_pos.z;

    return atan(plane_dist / height_diff);
}


double radian2Degree(const double &radian) {
    return radian * 180.0 / PI;
}

double degree2Radian(const double &degree) {
    return degree / 180.0 * PI;
}

double getDistance(Ptr<Node> AP, MyUeNode &UE_node) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();

    Vector UE_pos = UE_node.getPosition();

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;
    double dz = AP_pos.z - UE_pos.z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}


/*
    VLC SINR
*/
void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix) {
    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			for (int k = 1; k < effective_subcarrier_num+1; k++) {
                VLC_SINR_matrix[i][j][k] = estimateOneVlcSINR(VLC_LOS_matrix, i, j, k);
			}
		}
	}
}

double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double interference = 0;
    for (int i = 0; i < VLC_AP_num; i++) {
        if (i != VLC_AP_index)
            interference += pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2);
    }

    double noise = pow(optical_to_electric_power_ratio, 2) * VLC_AP_bandwidth * noise_power_spectral_density;
    double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[VLC_AP_index][UE_index] * estimateOneVlcFrontEnd(subcarrier_index), 2) / (interference + noise);

    // change to logarithmic SINR
    return (SINR == 0) ? 0 : 10*log10(SINR);
}

// front-end
// H_F(k) = exp( -(k * modulation_bandwidth) / (subcarrier_num*fitting_coefficient*3dB_cutoff))
double estimateOneVlcFrontEnd(int subcarrier_index) {
    return exp((-1) * subcarrier_index * VLC_AP_bandwidth / (subcarrier_num * fitting_coefficient * three_dB_cutoff));
}

/*
    VLC data rate
*/
void calculateAllVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix) {
    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			for (int k = 1; k < effective_subcarrier_num+1; k++) {
                VLC_data_rate_matrix[i][j][k] = estimateOneVlcDataRate(VLC_SINR_matrix, i, j, k);
			}
		}
	}
}

// data rate of the RU on the certain subcarrier based on (6)
double estimateOneVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double numerator = 2 * VLC_AP_bandwidth * getSpectralEfficiency(VLC_SINR_matrix[VLC_AP_index][UE_index][subcarrier_index]);
    double denominator = subcarrier_num * time_slot_num;

    return numerator / denominator;
}


// return the corresponding spectral efficiency of the given SINR according to the pre-established table
double getSpectralEfficiency(double SINR) {
    auto it = SINR_to_spectral_efficiency.lower_bound(SINR);

    return it->second;
}


/*
    RF data rate for each UE connected to WiFi

    NOTE:
    - Slot time is not given in the benchmark.
    - RTS/CTS is much shorter than SIFS, PIFS, and DIFS in the benchmark.
    However, the situation is opposite in "Downlink and uplink resource allocation in IEEE 802.11 wireless LANs".
*/
void calculateRfDataRate(std::vector<double> &RF_data_rate_vector) {
    // for the case when no serving UE
    RF_data_rate_vector[0] = 0.0;

    for (int serving_UE_num = 1; serving_UE_num < UE_num+1; serving_UE_num++) {
        double downlink_utilization_eff = calculateRfDownlinkUtilizationEfficiency(serving_UE_num);

        RF_data_rate_vector[serving_UE_num] = channel_bit_rate * downlink_utilization_eff / serving_UE_num;
    }
}

double calculateRfDownlinkUtilizationEfficiency(int serving_UE_num) {
    double system_utilization = calculateRfSystemUtilization(serving_UE_num);

    return system_utilization * utilization_ratio / (1 + utilization_ratio);
}

double calculateRfSystemUtilization(int serving_UE_num) {
    double t_c = RTS_time + DIFS_time;
    double t_s = RTS_time + CTS_time + header_time + propagation_delay + ACK_time + 3*SIFS_time + DIFS_time;
    double t_d = header_time + propagation_delay + ACK_time + SIFS_time + PIFS_time;

    double p_c = 2.0 / (max_backoff_stage + 1);
    double p_t = 1 - pow(1 - p_c, serving_UE_num + 1);
    double p_s = ((serving_UE_num + 1) * p_c * pow(1 - p_c, serving_UE_num)) / (p_t);
    double p_d = (serving_UE_num - 1) / (2 * serving_UE_num * p_s);

    double denominator = (1 - p_t) * slot_time;
    denominator += p_t * p_s * (1 - p_d) * t_s;
    denominator += p_t * p_s * p_d * t_d;
    denominator += p_t * (1 - p_s) * t_c;

    return (p_s * p_t * propagation_delay) / (denominator);
}
