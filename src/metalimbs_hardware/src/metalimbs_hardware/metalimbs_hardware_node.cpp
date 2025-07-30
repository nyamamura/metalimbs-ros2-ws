#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "metalimbs_hardware/hardware.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "metalimbs_hardware_node");
    ros::NodeHandle nh;
    ros::NodeHandle metalimbs_nh("/metalimbs");

    metalimbs_hardware::Hardware metalimbs;
    controller_manager::ControllerManager controller_manager(&metalimbs, metalimbs_nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(10);
    ros::Time prev_time = ros::Time::now();
    while (ros::ok()) {
        const ros::Time now = ros::Time::now();
        const ros::Duration dt = now - prev_time;
        prev_time = now;
        metalimbs.read(now, dt);
        controller_manager.update(now, dt);
        metalimbs.write(now, dt);
        rate.sleep();
    }
    spinner.stop();

    return 0;
}
