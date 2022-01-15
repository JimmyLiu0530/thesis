#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>

#define DEBUG_MODE 0
#define PI 3.14159265359
#define PROPOSED_METHOD 0

const double room_size = 16;
const int RF_AP_num = 1;
const int VLC_AP_num = 16;
const int VLC_AP_per_row = 4;
const int UE_num = 200;
const double time_period = 500; // ms
const double avg_speed = 1.0;
const double pause_time = 0.0;

/*
    RF AP
*/
const double RF_AP_height = 3.5;


/*
    VLC AP
*/
const double VLC_AP_height = 3.5;
const int VLC_AP_power = 9;
const int VLC_AP_bandwidth = 40; // MHz
const double noise_power_spectral_density = 1e-19;  //N_0
const double conversion_efficiency = 0.53; // optical to electrical conversion efficiency
const double optical_to_electric_power_ratio = 3; // κ

// these two values are found in "Resource Allocation in LiFi OFDMA Systems"
const int subcarrier_num = 64; // M = 64
const int effective_subcarrier_num = subcarrier_num / 2 - 1; // M_e = M/2 - 1
const int time_slot_num = 20; // K


/*
    UE
*/
const double UE_height = 1.5;
const double high_demand = 40; // Mbps
const double low_demand = 10; // Mbps
const double HDU_proportion = 0.5; // proportion of high-demand user

/*
    VLC channel
*/
const double field_of_view = 180;
const int PHI_half = 60; // semi-angle at half-illumination (phi_1/2)
const int filter_gain = 1;
const double refractive_index = 1.5;
const double receiver_area = 0.0001; // m^2
const double reflection_efficiency = 0.75;
const double fitting_coefficient = 2.88;
const double three_dB_cutoff = 30 // MHz

/*
    handover
*/
const double VHO_efficiency = 0.6;
const double HHO_efficiency = 0.9;

/*
    RF channel
*/
const double channel_bit_rate = 1732; // Mbps
const int max_backoff_stage = 1024;
const double RTS_time = 0.16; // µs
const double CTS_time = 0.14;
const double header_time = 0.23;
const double ACK_time = 0.14;
const double SIFS_time = 28.0;
const double PIFS_time = 80.0;
const double DIFS_time = 128.0;
const double slot_time = 52.0; // It is not given in benchmark paper. I infer this value by PIFS = SIFS + slot time, which is defined in ieee 802.11.
const double propagation_delay = 1.0;
const double utilization_ratio = 2.0; // ε


/*
    random orientation angle
*/
const double coherence_time = 130.0 // ms
const double sampling_time = 13.0 // ms
const double angle_mean = 30.0 // degree
const double angle_variance = 7.78 // degree
const double c_1 = pow(0.05, sampling_time/coherence_time);
const double c_0 = (1 - c_1) * angle_mean;
const double noise_variance = (1 - c_1 * c_1) * angle_variance * angle_variance;


/*
    utility function
*/
const double beta = 1.0; // fairness coefficient

#endif // GLOBAL_CONFIGURATION_H
