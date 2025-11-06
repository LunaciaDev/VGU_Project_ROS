#include "ros/duration.h"
#include "ros_control_boilerplate/sim_hw_interface.h"

class UR10eHWInterface : public ros_control_boilerplate::SimHWInterface {
   public:
    UR10eHWInterface(
        ros::NodeHandle& node_handle,
        urdf::Model*     urdf_model = NULL
    );

    virtual void init();
    virtual void read(ros::Duration& elapsed_time);
    virtual void write(ros::Duration& elapsed_time);
    virtual void enforceLimits(ros::Duration& period);

   private:
    virtual void positionControlSimulation(
        ros::Duration&    elapsed_time,
        const std::size_t joint_id
    );
};