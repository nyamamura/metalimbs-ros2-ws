#include <ros/ros.h>

#include "metalimbs_hardware/arm_control_service.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control_server");
    ros::NodeHandle nh;

    std::string port = "/dev/ttyUSB0";
    {
        ros::NodeHandle pnh("~");
        pnh.getParam("port", port);
    }

    metalimbs_hardware::ArmControlService arm_control_service(port);
    ros::Duration(5.0).sleep();

    ros::spin();

    return 0;
}
