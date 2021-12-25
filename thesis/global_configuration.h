#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <map>



#define DEBUG_MODE 0
#define PI 3.14159265359
#define PROPOSED_METHOD 1

const double room_size = 40;
const int RF_AP_num = 4;
const int VLC_AP_num = 16;
const int VLC_AP_per_row = 4;
const int RF_AP_per_row = 2;
const int UE_num = 30;
const double time_period = 500; // ms

/*
    RF AP
*/
const double RF_AP_height = 3.5;
const int RE_AP_power = 1;
const int RF_AP_bandwidth = 20; // MHz

/*
    VLC AP
*/
const double VLC_AP_height = 3.5;
const int VLC_AP_power = 9;
const int VLC_AP_bandwidth = 40; // MHz
const double noise_power_spectral_density = 1e-15;
const double conversion_efficiency = 0.53; // optical to electrical conversion efficiency
const double optical_to_electric_power_ratio = 3;

/*
    UE
*/
const double UE_height = 0.85;
const double high_demand = 40; //Mbps
const double low_demand = 10; //Mbps
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
const int subcarrier_num = 4;
const int time_slot_num = 16;
const double fitting_coefficient = 2.88;
const double 3dB_cutoff = 30 // MHz



#endif // GLOBAL_CONFIGURATION_H
