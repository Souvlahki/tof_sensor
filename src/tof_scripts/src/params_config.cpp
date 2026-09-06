#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <control_msgs/msg/dynamic_interface_group_values.hpp>

#include <string>
#include <vector>
#include <mutex>

class GpioSettingsPublisher : public rclcpp::Node
{
public:
    GpioSettingsPublisher() : Node("gpio_settings_publisher")
    {
        this->declare_parameter<double>("mode", 1.0);
        this->declare_parameter<double>("calibration", 16.0);
        this->declare_parameter<double>("sample_rate", 10.0);
        this->declare_parameter<double>("needs_calibration", 0.0);
        this->declare_parameter<double>("kp", 0.035);
        this->declare_parameter<double>("ki", 0.007);

        this->declare_parameter<std::string>("gpio_command_topic", "/gpio_controller/commands");
        this->declare_parameter<std::string>("tof_gpio_name", "base_tof_sensor_1_settings");
        this->declare_parameter<std::string>("pid_gpio_name", "pid_settings");

        gpio_command_topic_ = this->get_parameter("gpio_command_topic").as_string();
        tof_gpio_name_ = this->get_parameter("tof_gpio_name").as_string();
        pid_gpio_name_ = this->get_parameter("pid_gpio_name").as_string();

        mode_ = this->get_parameter("mode").as_double();
        calibration_ = this->get_parameter("calibration").as_double();
        sample_rate_ = this->get_parameter("sample_rate").as_double();
        needs_calibration_ = this->get_parameter("needs_calibration").as_double();
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();

        publisher_ = this->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
            gpio_command_topic_, rclcpp::SystemDefaultsQoS());

        // Publish the initial state once so the GPIO starts in a known
        // configuration, then react to changes from here on.
        publish_gpio(true, true);

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&GpioSettingsPublisher::on_parameter_change, this, std::placeholders::_1));
    }

private:
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool tof_changed = false;
        bool pid_changed = false;

        std::lock_guard<std::mutex> lock(state_mutex_);

        for (const auto &p : parameters)
        {
            const std::string &name = p.get_name();
            if (name == "mode")
            {
                mode_ = p.as_double();
                tof_changed = true;
            }
            else if (name == "calibration")
            {
                calibration_ = p.as_double();
                tof_changed = true;
            }
            else if (name == "sample_rate")
            {
                sample_rate_ = p.as_double();
                tof_changed = true;
            }
            else if (name == "needs_calibration")
            {
                needs_calibration_ = p.as_double();
                tof_changed = true;
            }
            else if (name == "kp")
            {
                kp_ = p.as_double();
                pid_changed = true;
            }
            else if (name == "ki")
            {
                ki_ = p.as_double();
                pid_changed = true;
            }
        }

        if (tof_changed || pid_changed)
        {
            publish_gpio(tof_changed, pid_changed);
        }

        return result;
    }

    // Publishes the current cached state. Only includes the GPIO groups
    // whose members actually changed, but always sends ALL interfaces
    // within a changed group so nothing on that GPIO is left undefined.
    void publish_gpio(bool include_tof, bool include_pid)
    {
        control_msgs::msg::DynamicInterfaceGroupValues msg;
        msg.header.stamp = this->now();

        if (include_tof)
        {
            control_msgs::msg::InterfaceValue tof_iv;
            tof_iv.interface_names = {"mode", "calibration", "sample_rate", "needs_calibration"};
            tof_iv.values = {mode_, calibration_, sample_rate_, needs_calibration_};
            msg.interface_groups.push_back(tof_gpio_name_);
            msg.interface_values.push_back(tof_iv);
        }

        if (include_pid)
        {
            control_msgs::msg::InterfaceValue pid_iv;
            pid_iv.interface_names = {"Kp", "Ki"};
            pid_iv.values = {kp_, ki_};
            msg.interface_groups.push_back(pid_gpio_name_);
            msg.interface_values.push_back(pid_iv);
        }

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published GPIO command update to %s", gpio_command_topic_.c_str());
    }
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr publisher_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::string gpio_command_topic_;
    std::string tof_gpio_name_;
    std::string pid_gpio_name_;

    std::mutex state_mutex_;
    double mode_{0.0};
    double calibration_{0.0};
    double sample_rate_{0.0};
    double needs_calibration_{0.0};
    double kp_{0.0};
    double ki_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpioSettingsPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}