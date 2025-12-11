#include <ur_client_library/rtde/data_package.h>

#include <exception>
#include <memory>

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "rtde_echo/RtdeData.h"
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/rtde/rtde_client.h"

static const std::vector<std::string> INPUT_RECIPE = {
    // RTDEClient expect the input_recipe to be non_empty (contrary to docs), so this is something the driver is not using
    "external_force_torque"
};
static const std::vector<std::string> OUTPUT_RECIPE = {
    // Require PolyscopeX, we have Polyscope 5.x
    "actual_robot_energy_consumed", "actual_robot_braking_energy_dissipated"
};
static const std::chrono::milliseconds READ_TIMEOUT{100};

int                                    main(int argc, char** argv) {
    ROS_INFO("Starting rtde_echo node");

    ros::init(argc, argv, "rtde_echo");
    ros::NodeHandle node_handle;

    std::string     robot_ip;
    if (!node_handle.getParam("/ur_hardware_interface/robot_ip", robot_ip)) {
        // fallback to the default IP from ursim
        robot_ip = "192.168.56.101";
    }

    // 1 thread for the publisher
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Register publisher for the data received
    ros::Publisher rtde_data_publisher = node_handle.advertise<rtde_echo::RtdeData>("/unity_bridge/rtde_data", 0);

    urcl::comm::INotifier            notifier;
    urcl::rtde_interface::RTDEClient client(
        robot_ip, notifier, OUTPUT_RECIPE, INPUT_RECIPE, 50
    );

    client.init();
    client.start();

    while (!ros::isShuttingDown()) {
        std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg =
            client.getDataPackage(READ_TIMEOUT);

        if (data_pkg) {
            double energy_consumed, braking_energy;

            data_pkg->getData<double>(OUTPUT_RECIPE[0], energy_consumed);
            data_pkg->getData<double>(OUTPUT_RECIPE[1], braking_energy);

            rtde_echo::RtdeData rtde_data;
            rtde_data.braking_energy_dissipated = braking_energy;
            rtde_data.energy_consumed = energy_consumed;

            rtde_data_publisher.publish(rtde_data);
        }
    }

    spinner.stop();
    return 0;
}