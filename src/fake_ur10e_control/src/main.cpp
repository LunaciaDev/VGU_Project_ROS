#include <memory>
#include "ros/spinner.h"
#include "ros_control_boilerplate/generic_hw_control_loop.h"
#include "ur10e_fake_interface.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur10e_fake_interface");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    std::shared_ptr<UR10eHWInterface> ur10e_hw_interface(new UR10eHWInterface(node_handle));
    ur10e_hw_interface->init();

    ros_control_boilerplate::GenericHWControlLoop control_loop(node_handle, ur10e_hw_interface);
    // this block until shutdown signal, so no need to handle this ourselves.
    control_loop.run();

    return 0;
}