#ifndef TOF_HARDWARE_INTERFACE_HPP
#define TOF_HARDWARE_INTERFACE_HPP

#include "hardware_interface/sensor.hpp"

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include "tof_serial.hpp"

namespace tof_hardware
{

    class TofHardwareInterface : public hardware_interface::SensorInterface
    {
    public:
        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        TofSerialConfig cfg_;
        TofSerial serial_;
    };

} // namespace tof_hardware

#endif // TOF_HARDWARE_INTERFACE_HPP