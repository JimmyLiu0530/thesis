#include <cmath>


#include "channel.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"



double radian2Degree(const double &radian) {
    return radian * 180 / PI;
}

double degree2Radian(const double &degree) {
    return degree * PI / 180;
}

double getDistance(Ptr<Node> AP, Ptr<Node> UE) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Ptr<MobilityModel> UE_mobility_model = UE->GetObject<MobilityModel>();

    return AP_mobility_model->GetDistanceFrom(UE_mobility_model);
}

/*
       plane_dist
    AP----------
      \        |
       \       |
        \      |
         \     | height_diff
          \    |
           \   |
            \θ |
             \ |
              UE

    arctan(plane_dist / height_diff) = θ
*/
double getIncidenceAngle(Ptr<Node> AP, Ptr<Node> UE) {
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

void calculateAllRfChannelGain(NodeContainer &RF_AP_nodes, NodeContainer &UE_nodes, std::vector<std::vector<double>> &RF_channel_gain_matrix) {
    for (int i = 0; i < RF_AP_num; i++)
	{
		for (int j = 0; j < UE_num; j++)
		{
			RF_channel_gain_matrix[i][j] = estimateOneRfChannelGain(RF_AP_nodes.Get(i), UE_nodes.Get(j));
		}
	}
}

void calculateAllVlcChannelGain(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes, std::vector<std::vector<double>> &VLC_channel_gain_matrix) {
    for (int i = 0; i < VLC_AP_num; i++)
	{
		for (int j = 0; j < UE_num; j++)
		{
			VLC_channel_gain_matrix[i][j] = estimateOneVlcChannelGain(VLC_AP_nodes.Get(i), UE_nodes.Get(j));
		}
	}
}

double estimateOneRfChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE) {

}

double estimateOneVlcChannelGain(Ptr<Node> VLC_AP, Ptr<Node> UE) {
    const double incidence_angle = getIncidenceAngle(VLC_AP, UE);
    if (radian2Degree(incidence_angle) >= field_of_view / 2)
        return 0.0;

    const double concentrator_gain = pow(refractive_index, 2) / pow(sin(degree2Radian(field_of_view/2)), 2);
    const double lambertian_coefficient = (-1) * log(2) / (log(cos(degree2Radian(PHI_half))));
    const double irradiance_angle = incidence_angle; // the angle of
    const double distance = getDistance(VLC_AP, UE);

    double channel_gain = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    channel_gain = channel_gain * concentrator_gain;
    channel_gain = channel_gain * filter_gain;
    channel_gain = channel_gain * pow(cos(irradiance_angle), lambertian_coefficient);
    channel_gain = channel_gain * cos(incidence_angle);

    return channel_gain;
}

void calculateAllRfSINR(std::vector<std::vector<double>> &RF_channel_gain_matrix, std::vector<std::vector<double>> &RF_SINR_matrix) {
    for (int i = 0; i < RF_AP_num; i++)
	{
		for (int j = 0; j < UE_num; j++)
		{
			RF_SINR_matrix[i][j] = estimateOneRfSINR(RF_channel_gain_matrix, i, j);
		}
	}
}

void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_channel_gain_matrix, std::vector<std::vector<double>> &VLC_SINR_matrix) {
    for (int i = 0; i < VLC_AP_num; i++)
	{
		for (int j = 0; j < UE_num; j++)
		{
			VLC_SINR_matrix[i][j] = estimateOneVlcSINR(VLC_channel_gain_matrix, i, j);
		}
	}
}

double estimateOneRfSINR(std::vector<std::vector<double>> &RF_channel_gain_matrix, int RF_AP_index, int UE_index) {

}

double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_channel_gain_matrix, int VLC_AP_index, int UE_index) {
    double interference = 0;
    for (int i = 0; i < VLC_AP_num; i++) {
        if (i != VLC_AP_index)
            interference += pow(conversion_efficiency * VLC_AP_power * VLC_channel_gain_matrix[i][UE_index], 2);
    }

    double noise = VLC_AP_bandwidth * noise_power_spectral_density;
    double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_channel_gain_matrix[VLC_AP_index][UE_index], 2) / (interference+noise);

    return SINR;
}

void calculateAllRfDataRate(std::vector<std::vector<double>> &RF_SINR_matrix, std::vector<std::vector<double>> &RF_data_rate_matrix);
void calculateAllVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, std::vector<std::vector<double>> &VLC_data_rate_matrix);

void calculateBitErrorRatio(std::vector<std::vector<double>> &VLC_SINR_matrix, std::vector<double> &bit_error_ratios) {
    for (int i = 0; i < VLC_AP_num; i++) {
        double first_term = 2 * sqrt(modulation_order-1) / sqrt(modulation_order * log(modulation_order));
        double second_term = erfc(sqrt(3*VLC_SINR_matrix[])) / (2 * modulation_order);

        bit_error_ratios[i] =
    }
}

