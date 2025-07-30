#pragma once

#include <optional>
#include <string>
#include <unistd.h>

#include <boost/asio.hpp>
#include <ros/ros.h>

namespace metalimbs_hardware
{

class Serial
{
public:
    Serial(const std::string& device_name);
    Serial(Serial&& serial);
    ~Serial();

    bool write(const std::string& data);
    template <class T>
    std::optional<T> read()
    {
        using namespace boost;
        T result;
        system::error_code error;
        asio::read(port_, boost::asio::buffer(reinterpret_cast<char*>(&result), sizeof(result)), error);
        if (error and error != boost::asio::error::eof) {
            ROS_WARN_STREAM("receive failed: " << error.message());
            return std::nullopt;
        }
        return result;
    }

private:
    const std::string device_name_;
    boost::asio::io_service io_;
    boost::asio::serial_port port_;
};
}  // namespace metalimbs_hardware
