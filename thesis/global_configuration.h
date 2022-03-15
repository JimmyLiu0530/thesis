#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>
#include <boost/math/constants/constants.hpp>

#define DEBUG_MODE 0
#define PROPOSED_METHOD 0

const double PI = boost::math::constants::pi<double>();
const double room_size = 16;
const int RF_AP_num = 1;
const int VLC_AP_num = 16;
const int VLC_AP_per_row = 4;
const int UE_num = 200;
const double time_period = 0.5; // s
const double avg_speed = 1.0;
const double pause_time = 0.0;
const int state_num = 1000;


/*
    RF AP
*/
const double RF_AP_height = 3.5; // m

/*
    VLC AP
*/
const double VLC_AP_height = 3.5;
const int VLC_AP_power = 9;
const int VLC_AP_bandwidth = 300; // MHz
const double noise_power_spectral_density = 1e-13;  //N_0 = 1e-19 A^2/Hz = 1e-13 A^2/MHz
const double conversion_efficiency = 0.53; // optical to electrical conversion efficiency
const double optical_to_electric_power_ratio = 3.0; // κ

// these two values are found in "Resource Allocation in LiFi OFDMA Systems"
const int subcarrier_num = 64; // M = 64
const int effective_subcarrier_num = subcarrier_num / 2 - 1; // M_e = M/2 - 1
const int time_slot_num = 20; // K


/*
    UE
*/
const double UE_height = 1.5;

// minimum acceptable discount on user's demand
//const double min_discount = 0.7;


/*
    VLC channel
*/
const double field_of_view = 180.0; // degree
const double PHI_half = 60.0; // semi-angle at half-illumination in degree
const double filter_gain = 1.0;
const double refractive_index = 1.5;
const double receiver_area = 1e-4; // 1 cm^2 = 0.0001 m^2
const double reflection_efficiency = 0.75;
const double fitting_coefficient = 2.88;
const double three_dB_cutoff = 30; // MHz

/*
    handover
*/
const double VHO_efficiency = 0.6;
const double HHO_efficiency = 0.9;

/*
    RF channel
*/
const int max_backoff_stage = 1024;
const double channel_bit_rate = 1732; // Mbps
const double RTS_time = 160.0; // µs
const double CTS_time = 140.0; // µs
const double header_time = 230.0; // µs
const double ACK_time = 140.0; // µs
const double SIFS_time = 28.0; // µs
const double PIFS_time = 80.0; // µs
const double DIFS_time = 128.0; // µs
const double slot_time = 52.0; // It is not given in benchmark paper. I infer this value by PIFS = SIFS + slot time based on ieee 802.11.
const double propagation_delay = 1000.0; // µs
const double utilization_ratio = 2.0; // ε


/*
    random orientation angle
*/
const double coherence_time = 130.0; // ms
const double sampling_time = 13.0; // ms
const double angle_mean = 30.0; // degree
const double angle_variance = 7.78; // degree
const double c_1 = pow(0.05, sampling_time/coherence_time);
const double c_0 = (1.0 - c_1) * angle_mean;
const double noise_variance = (1.0 - c_1 * c_1) * angle_variance * angle_variance;


/*
    utility function
*/
const double beta = 1.0; // fairness coefficient


/*
    weight in APA
*/
const double RU_throughput_weight = 0.8;


/*
    the unit of adjustment on demand discounting ratio
*/
const double delta_p = 0.05;

/*
    the period of the complete configuration (in states)
*/
const int complete_config_period = 5;

#endif // GLOBAL_CONFIGURATION_H
