#include "tof_hardware/tof_hardware_interface.hpp"
#include <cmath>

namespace tof_hardware
{
    hardware_interface::CallbackReturn
    TofHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {

        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        const auto &info_ = params.hardware_info;

        cfg_.port = info_.hardware_parameters.at("port");
        cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    TofHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        set_state("base_tof_sensor_1_settings/mode", SENSOR_INIT_MODE);
        set_state("base_tof_sensor_1_settings/calibration", SENSOR_INIT_CALIBRATION);
        set_state("base_tof_sensor_1_settings/sample_rate", SENSOR_INIT_SAMPLE_RATE_HZ);
        set_state("base_tof_sensor_1_settings/needs_calibration", static_cast<double>(0.0));

        set_state("base_left_wheel_joint/velocity", static_cast<double>(0.0));
        set_state("base_right_wheel_joint/velocity", static_cast<double>(0.0));
        set_state("base_left_wheel_joint/position", static_cast<double>(0.0));
        set_state("base_right_wheel_joint/position", static_cast<double>(0.0));

        set_state("pid_settings/Kp", INITIAL_Kp);
        set_state("pid_settings/Ki", INITIAL_Ki);

        for (int i = 0; i < MOTOR_SAYISI; i++)
        {

            cached_motor_data_[i].last_tick = 0;
        }
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

        TxFrame packet;
        TofReadStatus status;
        while ((status = serial_.ReadPacket(packet)) == TofReadStatus::Packet)
        {

            set_state("base_tof_sensor_1_joint/range", static_cast<double>(packet.tof_packet[0].range) / 1000.0);

            for (int i = 0; i < MOTOR_SAYISI; i++)
            {
                int32_t new_total = packet.motor_packet[i].ticks;                    
                int32_t delta_ticks = new_total - cached_motor_data_[i].last_tick; 
                cached_motor_data_[i].last_tick = new_total;                       

                double if_vel = static_cast<double>(delta_ticks) / ENCODER_TICKS / REDUCTION_RATION / period.seconds();
                double if_pose = static_cast<double>(new_total) / ENCODER_TICKS / REDUCTION_RATION;

                if (packet.motor_packet[i].motor_id == 0)
                {
                    set_state("base_left_wheel_joint/velocity", if_vel);
                    set_state("motor_data/left_calculated_vel", if_vel);
                    set_state("base_left_wheel_joint/position", if_pose);
                }
                else
                {
                    set_state("base_right_wheel_joint/velocity", if_vel); 
                    set_state("base_right_wheel_joint/position", if_pose);
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    TofHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        bool tof_changed = false;
        bool pid_changed = false;
        double mode = get_command("base_tof_sensor_1_settings/mode");
        double calibration = get_command("base_tof_sensor_1_settings/calibration");
        double sample_rate = get_command("base_tof_sensor_1_settings/sample_rate");
        double needs_calibration = get_command("base_tof_sensor_1_settings/needs_calibration");

        double kp = get_command("pid_settings/Kp");
        double ki = get_command("pid_settings/Ki");

        // RCLCPP_INFO(rclcpp::get_logger("debug"), "%lf %lf %lf %lf", mode, calibration, sample_rate, needs_calibration);

        double last_mode = get_state("base_tof_sensor_1_settings/mode");
        double last_calibration = get_state("base_tof_sensor_1_settings/calibration");
        double last_sample_rate = get_state("base_tof_sensor_1_settings/sample_rate");
        double last_needs_calibration = get_state("base_tof_sensor_1_settings/needs_calibration");
        double last_kp = get_state("pid_settings/Kp");
        double last_ki = get_state("pid_settings/Ki");
        // RCLCPP_INFO(rclcpp::get_logger("debug"), "last: %lf %lf %lf %lf", last_mode, last_calibration, last_sample_rate, last_needs_calibration);

        mode = std::isfinite(mode) ? mode : last_mode;
        calibration = std::isfinite(calibration) ? calibration : last_calibration;
        sample_rate = std::isfinite(sample_rate) ? sample_rate : last_sample_rate;
        needs_calibration = std::isfinite(needs_calibration) ? needs_calibration : last_needs_calibration;
        kp = std::isfinite(kp) ? kp : last_kp;
        ki = std::isfinite(ki) ? ki : last_ki;


        tof_changed =
            mode != last_mode ||
            calibration != last_calibration ||
            sample_rate != last_sample_rate || needs_calibration != last_needs_calibration;


        pid_changed = kp != last_kp || ki != last_ki;
            

        RxFrame frame{};
        frame.tof_settings.sensor_id = static_cast<uint8_t>(1);
        frame.tof_settings.mode = static_cast<uint16_t>(mode);
        frame.tof_settings.calibration = static_cast<int16_t>(calibration);
        frame.tof_settings.sample_rate = static_cast<uint16_t>(sample_rate);
        frame.tof_settings.needs_calibration = static_cast<bool>(needs_calibration);

        frame.motor_telemetry[0].motor_id = 0;
        frame.motor_telemetry[0].ticks = static_cast<int32_t>(get_command("base_left_wheel_joint/velocity") * REDUCTION_RATION * ENCODER_TICKS);
        set_state("motor_data/left_requested_vel", get_command("base_left_wheel_joint/velocity"));
        // RCLCPP_INFO(rclcpp::get_logger("send"), " sending ticks: %d ", frame.motor_telemetry[0].ticks);
        frame.motor_telemetry[1].motor_id = 1;
        frame.motor_telemetry[1].ticks = 0;

        frame.pid_settings.ki = static_cast<float>(ki);
        frame.pid_settings.kp = static_cast<float>(kp);

        if (tof_changed)
        {
            frame.tof_settings.has_changed = true;
        }

        if(pid_changed) {
            frame.pid_settings.has_changed = true;
        }

        // RCLCPP_INFO(rclcpp::get_logger("debug"), "frame %d %d %d %d %d", frame.tof_settings.mode, frame.tof_settings.calibration, frame.tof_settings.sample_rate, frame.tof_settings.needs_calibration, frame.tof_settings.has_changed);

        if (serial_.SendData(frame))
        {
            // RCLCPP_INFO(rclcpp::get_logger("send"), "sending data");
            if (tof_changed)
            {

                set_state("base_tof_sensor_1_settings/mode", static_cast<double>(mode));
                set_state("base_tof_sensor_1_settings/calibration", static_cast<double>(calibration));
                set_state("base_tof_sensor_1_settings/sample_rate", static_cast<double>(sample_rate));
            }

            if(pid_changed) {
                set_state("pid_settings/Kp", static_cast<double>(kp));
                set_state("pid_settings/Ki", static_cast<double>(ki));
            }
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tof_hardware::TofHardwareInterface, hardware_interface::SystemInterface)