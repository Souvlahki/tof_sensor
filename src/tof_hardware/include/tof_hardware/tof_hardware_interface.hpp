#ifndef TOF_HARDWARE_INTERFACE_HPP
#define TOF_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include <libserial/SerialPort.h>
#include "tof_serial.hpp"

#define SENSOR_INIT_MODE 1.0
#define SENSOR_INIT_CALIBRATION 16.0
#define SENSOR_INIT_SAMPLE_RATE_HZ 10.0

#define ENCODER_TICKS 6000
#define INITIAL_Kp 0.0035
#define INITIAL_Ki 0.007

namespace tof_hardware
{
    struct CachedMotorData
    {
        int32_t last_tick;
    };

    struct TofSensorInterface
    {
        std::string sensor_name; // e.g. "base_tof_sensor_1_joint"
        int sensor_id = -1;
    };

    class TofHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        TofSerialConfig cfg_;
        TofSerial serial_;

        CachedMotorData cached_motor_data_[MOTOR_SAYISI];
        std::vector<TofSensorInterface> tof_sensors_;
    };

} // namespace tof_hardware

#endif // TOF_HARDWARE_INTERFACE_HPP