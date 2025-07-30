#include <string>

#include "metalimbs_hardware/command.hpp"
#include "metalimbs_hardware/hardware.hpp"
#include "metalimbs_hardware/serial.hpp"

namespace metalimbs_hardware
{

Hardware::Hardware()
    : left_serial_(Serial("/dev/ttyUSB0")), right_serial_(Serial("/dev/ttyUSB1")), first(true)
{
    const std::string joints_name[7] = {
        "shoulder2_mount_joint",
        "shoulder2_joint",
        "upper_arm_joint",
        "elbow_joint",
        "forearm_joint",
        "wrist_joint",
        "handunit_mount_joint"};

    for (int i = 0; i < 7; ++i) {
        {
            hardware_interface::JointStateHandle state_handle("left_" + joints_name[i], &left_position_[i], &left_velocity_[i], &left_effort_[i]);
            joint_state_interface_.registerHandle(state_handle);
        }
        {
            hardware_interface::JointStateHandle state_handle("right_" + joints_name[i], &right_position_[i], &right_velocity_[i], &right_effort_[i]);
            joint_state_interface_.registerHandle(state_handle);
        }
    }
    registerInterface(&joint_state_interface_);

    for (int i = 0; i < 7; ++i) {
        {
            hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle("left_" + joints_name[i]), &left_command_[i]);
            joint_position_interface_.registerHandle(joint_handle);
        }
        {
            hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle("right_" + joints_name[i]), &right_command_[i]);
            joint_position_interface_.registerHandle(joint_handle);
        }
    }
    registerInterface(&joint_position_interface_);

    {
        command::SetTorqueState command;
        command.state = true;
        const auto bytes = command.serialize();
        const std::string buf{bytes.begin(), bytes.end()};
        left_serial_.write(buf);
        right_serial_.write(buf);
    }
}

void Hardware::write(const ros::Time&, const ros::Duration&)
{
    {
        command::SetAllPosition left_command;
        command::SetAllPosition right_command;
        for (int i = 0; i < 7; ++i) {
            left_command.positions[i] = int16_t(convert_coef * left_command_[i]);
            right_command.positions[i] = int16_t(convert_coef * right_command_[i]);
        }
        {
            const auto bytes = left_command.serialize();
            left_serial_.write(std::string(bytes.begin(), bytes.end()));
        }
        {
            const auto bytes = right_command.serialize();
            right_serial_.write(std::string(bytes.begin(), bytes.end()));
        }
    }
    {
        command::ReadAllPosition command;
        const auto bytes = command.serialize();
        const std::string buf{bytes.begin(), bytes.end()};
        left_serial_.write(buf);
        right_serial_.write(buf);
    }
}

void Hardware::read(const ros::Time&, const ros::Duration&)
{
    if (first) {
        first = false;
        return;
    }
    {
        const auto left_result = left_serial_.read<command::ReadAllPosition::Result>();
        const auto right_result = right_serial_.read<command::ReadAllPosition::Result>();
        if (left_result) {
            ROS_INFO("left result returned.");
            for (int i = 0; i < 7; ++i) {
                left_position_[i] = left_result->positions[i] / convert_coef;
            }
        }
        if (right_result) {
            ROS_INFO("right result returned.");
            for (int i = 0; i < 7; ++i) {
                right_position_[i] = right_result->positions[i] / convert_coef;
            }
        }
    }
}

}  // namespace metalimbs_hardware
