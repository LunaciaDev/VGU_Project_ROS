#include "ros/duration.h"
#include "ros_control_boilerplate/generic_hw_interface.h"

class UR10eHWInterface : public ros_control_boilerplate::GenericHWInterface {
    public:
    UR10eHWInterface(ros::NodeHandle& node_handle, urdf::Model* urdf_model = NULL);

    virtual void read(ros::Duration& elapsed_time);
    virtual void write(ros::Duration& elapsed_time);
    virtual void enforceLimits(ros::Duration& period);
};