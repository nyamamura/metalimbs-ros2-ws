#include "metalimbs_hardware/serial.hpp"

#include <boost/asio.hpp>
#include <ros/ros.h>

namespace metalimbs_hardware
{

Serial::Serial(const std::string& device_name)
    : device_name_(device_name), io_(), port_(io_, device_name_)
{
    using boost::asio::serial_port_base;
    port_.set_option(serial_port_base::baud_rate(3000000));
    port_.set_option(serial_port_base::character_size(8));
    port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
}

Serial::~Serial()
{
}

bool Serial::write(const std::string& data)
{
    using namespace boost;

    std::string buf = data;
    {
        std::ostringstream oss;
        for (const char c : buf) {
            oss << std::hex << "0x" << static_cast<int>(c) << " ";
        }
        ROS_INFO_STREAM("write: " << oss.str());
    }
    system::error_code error;
    asio::write(port_, asio::buffer(buf.c_str(), buf.size()), error);
    if (error) {
        ROS_WARN_STREAM("serial write failed: " << error.message());
        return false;
    }
    return true;
}

}  // namespace metalimbs_hardware
