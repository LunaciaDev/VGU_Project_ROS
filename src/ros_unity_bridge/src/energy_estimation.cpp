#include <boost/smart_ptr/shared_ptr.hpp>

#include "common.hpp"
#include "ros/topic.h"
#include "rtde_echo/RtdeData.h"

void start_energy_recording(struct EnergyData* data_package) {
    boost::shared_ptr<const rtde_echo::RtdeData> rtde_data;

    if (data_package->use_estimation) {
        rtde_data = ros::topic::waitForMessage<rtde_echo::RtdeData>(
            "/unity_bridge/unity_rtde_data"
        );
    } else {
        rtde_data = ros::topic::waitForMessage<rtde_echo::RtdeData>(
            "/unity_bridge/rtde_data"
        );
    }
    data_package->brake_energy = rtde_data->braking_energy_dissipated;
    data_package->move_energy = rtde_data->energy_consumed;
}

std::pair<double, double> stop_energy_recording(
    const struct EnergyData* data_package
) {
    boost::shared_ptr<const rtde_echo::RtdeData> rtde_data;

    if (data_package->use_estimation) {
        rtde_data = ros::topic::waitForMessage<rtde_echo::RtdeData>(
            "/unity_bridge/unity_rtde_data"
        );
    } else {
        rtde_data = ros::topic::waitForMessage<rtde_echo::RtdeData>(
            "/unity_bridge/rtde_data"
        );
    }

    return {
        rtde_data->energy_consumed - data_package->move_energy,
        rtde_data->braking_energy_dissipated - data_package->brake_energy
    };
}