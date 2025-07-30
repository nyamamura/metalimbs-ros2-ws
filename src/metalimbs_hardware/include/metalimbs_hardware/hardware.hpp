#pragma once

#include <array>
#include <optional>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "metalimbs_hardware/serial.hpp"

namespace metalimbs_hardware
{
class Hardware : public hardware_interface::RobotHW
{
public:
    Hardware();

    void write(const ros::Time& time, const ros::Duration& duration) override;
    void read(const ros::Time& time, const ros::Duration& duration) override;

private:
    static constexpr double convert_coef = 180. / M_PI * 100.;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface joint_position_interface_;

    std::array<double, 7> left_command_;
    std::array<double, 7> right_command_;
    std::array<double, 7> left_position_;
    std::array<double, 7> right_position_;
    std::array<double, 7> left_velocity_;
    std::array<double, 7> right_velocity_;
    std::array<double, 7> left_effort_;
    std::array<double, 7> right_effort_;

    Serial left_serial_;
    Serial right_serial_;

    bool first;
};
}  // namespace metalimbs_hardware
