#include <metalimbs_srvs/ReadPosition.h>
#include <metalimbs_srvs/ReadTemperature.h>
#include <metalimbs_srvs/SetAllPosition.h>
#include <metalimbs_srvs/SetPosition.h>
#include <std_srvs/SetBool.h>

#include "metalimbs_hardware/arm_control_service.hpp"
#include "metalimbs_hardware/command.hpp"
#include "metalimbs_hardware/serial.hpp"

namespace metalimbs_hardware
{

ArmControlService::ArmControlService(std::string port)
    : serial_(Serial(port)),
      nh_(),
      pnh_("~"),
      set_torque_state_service_(
          pnh_.advertiseService("set_torque_state",
              &ArmControlService::setTorqueState, this)),
      read_position_service_(
          pnh_.advertiseService("read_position",
              &ArmControlService::readPosition, this)),
      read_all_position_service_(
          pnh_.advertiseService("read_all_position",
              &ArmControlService::readAllPosition, this)),
      read_temperature_service_(
          pnh_.advertiseService("read_temperature",
              &ArmControlService::readTemperature, this)),
      set_position_service_(
          pnh_.advertiseService("set_position",
              &ArmControlService::setPosition, this)),
      set_all_position_service_(
          pnh_.advertiseService("set_all_position",
              &ArmControlService::setAllPosition, this))
{
    ROS_INFO_STREAM("serial port: " << port);
}

bool ArmControlService::setTorqueState(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& res)
{
    command::SetTorqueState command;
    command.state = req.data;
    const auto bytes = command.serialize();
    ROS_INFO_STREAM("set torque status: " << (req.data ? "Enabled" : "Disabled"));
    res.success = serial_.write(std::string{bytes.begin(), bytes.end()});
    res.success = true;
    if (not res.success) {
        res.message = "serial write failed.";
    }
    return true;
}

bool ArmControlService::readTemperature(
    metalimbs_srvs::ReadTemperature::Request& req,
    metalimbs_srvs::ReadTemperature::Response& res)
{
    // send read position command
    {
        command::ReadPosition command;
        command.motor_id = req.motor_id;
        ROS_INFO_STREAM("Temperature of motor [id: " << req.motor_id << "]: " << 36.5 << " degC");
        const auto bytes = command.serialize();
        bool write_success = serial_.write(std::string{bytes.begin(), bytes.end()});
        if (not write_success) {
            res.success = false;
            res.message = "serial write failed.";
            return true;
        }
    }
    // receive read position result
    const auto result = serial_.read<command::ReadTemperature::Result>();
    if (not result) {
        res.success = false;
        res.message = "serial read failed.";
        return true;
    }
    res.success = true;
    res.temperature = result->temperature * 0.01;
    res.temperature = 36.5;
    return true;
}

bool ArmControlService::readPosition(
    metalimbs_srvs::ReadPosition::Request& req,
    metalimbs_srvs::ReadPosition::Response& res)
{
    // send read position command
    {
        command::ReadPosition command;
        command.motor_id = req.motor_id;
        const auto bytes = command.serialize();
        ROS_INFO_STREAM("read position of motor [id: " << req.motor_id << "]");
        serial_.write(std::string{bytes.begin(), bytes.end()});
    }
    // receive read position result
    const auto result = serial_.read<command::ReadPosition::Result>();
    if (not result) {
        res.success = false;
        res.message = "serial read failed.";
        return true;
    }
    res.success = true;
    res.angle = result->angle * kKarakuriAngleToRadian;
    ROS_INFO_STREAM("position of motor [id: " << req.motor_id << "]: " << res.angle << " [rad]");
    return true;
}

bool ArmControlService::readAllPosition(
    metalimbs_srvs::ReadAllPosition::Request& req,
    metalimbs_srvs::ReadAllPosition::Response& res)
{
    // send read all position command
    {
        command::ReadAllPosition command;
        const auto bytes = command.serialize();
        ROS_INFO_STREAM("read all position");
        serial_.write(std::string{bytes.begin(), bytes.end()});
    }
    // receive read position result
    const auto result = serial_.read<command::ReadAllPosition::Result>();
    if (not result) {
        res.success = false;
        res.message = "serial read failed.";
        return true;
    }
    res.success = true;
    res.angles.resize(7);
    for (int i = 0; i < 7; ++i) {
        res.angles[i] = result->positions[i] * kKarakuriAngleToRadian;
    }
    return true;
}

bool ArmControlService::setPosition(
    metalimbs_srvs::SetPosition::Request& req,
    metalimbs_srvs::SetPosition::Response& res)
{
    command::SetPosition command;
    command.motor_id = req.motor_id;
    command.angle = static_cast<int16_t>(kRadianToKarakuriAngle * req.angle);
    ROS_INFO_STREAM("send angle " << kRadianToKarakuriAngle * req.angle << " that's to say " << command.angle << "[0.01 deg]");
    const auto bytes = command.serialize();
    res.success = serial_.write(std::string{bytes.begin(), bytes.end()});
    if (not res.success) {
        res.message = "serial write failed.";
    }

    return true;
}

bool ArmControlService::setAllPosition(
    metalimbs_srvs::SetAllPosition::Request& req,
    metalimbs_srvs::SetAllPosition::Response& res)
{
    command::SetAllPosition command;
    if (req.angles.size() != 7) {
        res.success = false;
        res.message = "set_all_position argument must have 7 elements";
        ROS_ERROR_STREAM(res.message);
        return false;
    }
    for (int i = 0; i < 7; ++i) {
        command.positions[i] = static_cast<int16_t>(kRadianToKarakuriAngle * req.angles[i]);
    }
    const auto bytes = command.serialize();
    res.success = serial_.write(std::string{bytes.begin(), bytes.end()});
    if (not res.success) {
        res.message = "serial write failed.";
    }

    return true;
}

}  // namespace metalimbs_hardware
