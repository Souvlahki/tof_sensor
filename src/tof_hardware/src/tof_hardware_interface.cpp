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

        for (const auto &sensor : info_.sensors)
        {
            auto it = sensor.parameters.find("id");
            if (it == sensor.parameters.end())
            {
                RCLCPP_WARN(rclcpp::get_logger("TofHardwareInterface"),
                            "Sensor '%s' has no 'id' param, skipping", sensor.name.c_str());
                continue;
            }

            TofSensorInterface tsi;
            tsi.sensor_name = sensor.name;
            tsi.sensor_id = std::stoi(it->second);

            tof_sensors_.push_back(tsi);
        }

        cfg_.port = info_.hardware_parameters.at("port");
        cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    TofHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        for (const auto &ts : tof_sensors_)
        {
            set_state(ts.sensor_name + "/range", 0.0);

            set_state(ts.sensor_name + "_settings/mode", SENSOR_INIT_MODE);
            set_state(ts.sensor_name + "_settings/calibration", SENSOR_INIT_CALIBRATION);
            set_state(ts.sensor_name + "_settings/sample_rate", SENSOR_INIT_SAMPLE_RATE_HZ);
            set_state(ts.sensor_name + "_settings/needs_calibration", 0.0);
        }

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
            for (int i = 0; i < SENSOR_COUNT; i++)
            {
                int sensor_id = packet.tof_packet[i].sensor_id;

                auto ts_it = std::find_if(tof_sensors_.begin(), tof_sensors_.end(),
                                          [sensor_id](const TofSensorInterface &ts)
                                          { return ts.sensor_id == sensor_id; });

                if (ts_it == tof_sensors_.end())
                    continue;

                set_state(ts_it->sensor_name + "/range", static_cast<double>(packet.tof_packet[i].range) / 1000.0);

                // settings-related telemetry (mode, calibration, sample_rate) mirrored back as state,
                // under the derived "_settings" name
                set_state(ts_it->sensor_name + "_settings/calibration", static_cast<double>(packet.tof_packet[i].calibration));
                set_state(ts_it->sensor_name + "_settings/mode", static_cast<double>(packet.tof_packet[i].mode));
                set_state(ts_it->sensor_name + "_settings/sample_rate", static_cast<double>(packet.tof_packet[i].sample_rate));
            }

            for (int i = 0; i < MOTOR_SAYISI; i++)
            {
                int32_t new_total = packet.motor_packet[i].ticks;
                int32_t delta_ticks = new_total - cached_motor_data_[i].last_tick;
                cached_motor_data_[i].last_tick = new_total;

                double if_vel = ((float)delta_ticks / ENCODER_TICKS) * 2.0 * M_PI / period.seconds();
                double if_pose = ((float)new_total / ENCODER_TICKS) * 2.0 * M_PI;

                if (packet.motor_packet[i].motor_id == 0)
                {
                    set_state("base_left_wheel_joint/velocity", if_vel);
                    set_state("motor_data/left_calculated_vel", if_vel);
                    set_state("base_left_wheel_joint/position", if_pose);
                }
                else
                {
                    set_state("base_right_wheel_joint/velocity", if_vel);
                    set_state("motor_data/right_calculated_vel", if_vel);
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

        double kp = get_command("pid_settings/Kp");
        double ki = get_command("pid_settings/Ki");

        double last_kp = get_state("pid_settings/Kp");
        double last_ki = get_state("pid_settings/Ki");

        kp = std::isfinite(kp) ? kp : last_kp;
        ki = std::isfinite(ki) ? ki : last_ki;

        pid_changed = kp != last_kp || ki != last_ki;

        RxFrame frame{};

        std::string changed_settings_name;
        double changed_mode = 0, changed_calibration = 0, changed_sample_rate = 0, changed_needs_calibration = 0;

        for (const auto &ts : tof_sensors_)
        {
            const std::string settings_name = ts.sensor_name + "_settings";

            double mode = get_command(settings_name + "/mode");
            double calibration = get_command(settings_name + "/calibration");
            double sample_rate = get_command(settings_name + "/sample_rate");
            double needs_calibration = get_command(settings_name + "/needs_calibration");

            double last_mode = get_state(settings_name + "/mode");
            double last_calibration = get_state(settings_name + "/calibration");
            double last_sample_rate = get_state(settings_name + "/sample_rate");
            double last_needs_calibration = get_state(settings_name + "/needs_calibration");

            mode = std::isfinite(mode) ? mode : last_mode;
            calibration = std::isfinite(calibration) ? calibration : last_calibration;
            sample_rate = std::isfinite(sample_rate) ? sample_rate : last_sample_rate;
            needs_calibration = std::isfinite(needs_calibration) ? needs_calibration : last_needs_calibration;

            bool changed = mode != last_mode || calibration != last_calibration ||
                           sample_rate != last_sample_rate || needs_calibration != last_needs_calibration;

            if (!changed)
                continue;

            frame.tof_settings.sensor_id = static_cast<uint8_t>(ts.sensor_id);
            frame.tof_settings.mode = static_cast<uint16_t>(mode);
            frame.tof_settings.calibration = static_cast<int16_t>(calibration);
            frame.tof_settings.sample_rate = static_cast<uint16_t>(sample_rate);
            frame.tof_settings.needs_calibration = static_cast<bool>(needs_calibration);

            changed_settings_name = settings_name;
            changed_mode = mode;
            changed_calibration = calibration;
            changed_sample_rate = sample_rate;
            changed_needs_calibration = needs_calibration;

            tof_changed = true;
            break; // only one sensor changes at a time
        }

        double right_cmd = get_command("base_right_wheel_joint/velocity");
        double left_cmd = get_command("base_left_wheel_joint/velocity");

        double right_vel = std::isfinite(right_cmd) ? right_cmd : 0.0;
        double left_vel = std::isfinite(left_cmd) ? left_cmd : 0.0;

        frame.motor_telemetry[0].motor_id = 0;
        frame.motor_telemetry[0].ticks = static_cast<int32_t>(
            left_vel / (2.0 * M_PI) * ENCODER_TICKS);

        set_state("motor_data/left_requested_vel", left_vel);

        frame.motor_telemetry[1].motor_id = 1;
        frame.motor_telemetry[1].ticks = static_cast<int32_t>(
            right_vel / (2.0 * M_PI) * ENCODER_TICKS);
        set_state("motor_data/right_requested_vel", right_vel);

        frame.pid_settings.ki = static_cast<float>(ki);
        frame.pid_settings.kp = static_cast<float>(kp);

        if (tof_changed)
        {
            frame.tof_settings.has_changed = true;
        }

        if (pid_changed)
        {
            frame.pid_settings.has_changed = true;
        }

        if (serial_.SendData(frame))
        {
            if (tof_changed)
            {
                set_state(changed_settings_name + "/mode", changed_mode);
                set_state(changed_settings_name + "/calibration", changed_calibration);
                set_state(changed_settings_name + "/sample_rate", changed_sample_rate);
                set_state(changed_settings_name + "/needs_calibration", changed_needs_calibration);
            }

            if (pid_changed)
            {
                set_state("pid_settings/Kp", static_cast<double>(kp));
                set_state("pid_settings/Ki", static_cast<double>(ki));
            }
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tof_hardware::TofHardwareInterface, hardware_interface::SystemInterface)