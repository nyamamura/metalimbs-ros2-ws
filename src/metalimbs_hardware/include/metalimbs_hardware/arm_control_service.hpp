#pragma once

#include <metalimbs_srvs/ReadAllPosition.h>
#include <metalimbs_srvs/ReadPosition.h>
#include <metalimbs_srvs/ReadTemperature.h>
#include <metalimbs_srvs/SetAllPosition.h>
#include <metalimbs_srvs/SetPosition.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "metalimbs_hardware/serial.hpp"

namespace metalimbs_hardware
{

class ArmControlService
{
public:
    ArmControlService(std::string port);

private:
    bool setTorqueState(
        std_srvs::SetBool::Request& req,
        std_srvs::SetBool::Response& res);

    bool readTemperature(
        metalimbs_srvs::ReadTemperature::Request& req,
        metalimbs_srvs::ReadTemperature::Response& res);

    bool readPosition(
        metalimbs_srvs::ReadPosition::Request& req,
        metalimbs_srvs::ReadPosition::Response& res);

    bool readAllPosition(
        metalimbs_srvs::ReadAllPosition::Request& req,
        metalimbs_srvs::ReadAllPosition::Response& res);

    bool setPosition(
        metalimbs_srvs::SetPosition::Request& req,
        metalimbs_srvs::SetPosition::Response& res);

    bool setAllPosition(
        metalimbs_srvs::SetAllPosition::Request& req,
        metalimbs_srvs::SetAllPosition::Response& res);

    static constexpr double kKarakuriAngleToRadian = M_PI / 180.0 * 0.01;
    static constexpr double kRadianToKarakuriAngle = 180.0 / M_PI * 100.0;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    metalimbs_hardware::Serial serial_;

    [[maybe_unused]] ros::ServiceServer set_torque_state_service_;
    [[maybe_unused]] ros::ServiceServer read_position_service_;
    [[maybe_unused]] ros::ServiceServer read_all_position_service_;
    [[maybe_unused]] ros::ServiceServer read_temperature_service_;
    [[maybe_unused]] ros::ServiceServer set_position_service_;
    [[maybe_unused]] ros::ServiceServer set_all_position_service_;
};

}  // namespace metalimbs_hardware
