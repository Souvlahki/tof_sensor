#include "tof_hardware/tof_hardware_interface.hpp"

namespace tof_hardware
{
    hardware_interface::CallbackReturn
    TofHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {

        if (hardware_interface::SensorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        const auto &info_ = params.hardware_info;

        cfg_.port = info_.hardware_parameters.at("port");
        cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    TofHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        try
        {
            serial_.Init(cfg_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TofHardwareInterface"),
                         "Failed to open %s @ %d: %s", cfg_.port.c_str(), cfg_.baud_rate, e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    TofHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        if (serial_.IsOpen())
            serial_.Close();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    TofHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        TofPacket packet;
        TofReadStatus status;

        while ((status = serial_.ReadPacket(packet)) == TofReadStatus::Packet)
        {

            set_state("base_tof_sensor_1_joint/range", static_cast<double>(packet.range) / 1000.0);
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tof_hardware::TofHardwareInterface, hardware_interface::SensorInterface)