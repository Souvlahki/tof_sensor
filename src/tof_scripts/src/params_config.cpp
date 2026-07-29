// tof_gpio_param_node.cpp
//
// Generic bridge: any ROS 2 parameter declared on this node (other than the
// node's own config parameters, e.g. gpio_group_name/commands_topic) is
// treated as a gpio command interface value. Editing it via `ros2 param set`
// or rqt_reconfigure publishes just that interface to gpio_controller's
// command topic as a control_msgs/msg/DynamicInterfaceGroupValues message.
//
// To add a new interface later (e.g. because you added a new
// <command_interface> to a <gpio> block in the URDF), you only need to add
// ONE line in the constructor -- a declare_parameter<T> call with a name
// matching the interface name exactly. Nothing else in this file needs to
// change.
//
// This node does NOT talk to the STM32 directly -- it only drives the
// gpio_controller, which in turn drives your hardware interface's write().

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

using control_msgs::msg::DynamicInterfaceGroupValues;
using control_msgs::msg::InterfaceValue;

class TofGpioParamNode : public rclcpp::Node
{
public:
    TofGpioParamNode()
        : Node("tof_gpio_param_node")
    {
        // Node's own configuration -- NOT forwarded as an interface value.
        // Any parameter name in reserved_params_ is excluded from forwarding.
        gpio_group_ = this->declare_parameter<std::string>(
            "gpio_group_name", "base_tof_sensor_1_settings");
        commands_topic_ = this->declare_parameter<std::string>(
            "commands_topic", "/gpio_controller/commands");
        reserved_params_ = {"gpio_group_name", "commands_topic"};

        state_sub_ = this->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
            "/gpio_controller/gpio_states", rclcpp::SystemDefaultsQoS(),
            std::bind(&TofGpioParamNode::OnHardwareState, this, std::placeholders::_1));

        // ---------------------------------------------------------------
        // Interface values. Add a new line here whenever you add a new
        // command_interface to the gpio block in your URDF -- the name
        // MUST match the interface name exactly (e.g. "mode" maps to
        // "<gpio_group_>/mode" on the wire). No other code change needed.
        // ---------------------------------------------------------------
        this->declare_parameter<int>("mode", 1);
        this->declare_parameter<int>("calibration", 65515);
        this->declare_parameter<int>("sample_rate", 10);
        this->declare_parameter<bool>("needs_calibration", false);

        publisher_ = this->create_publisher<DynamicInterfaceGroupValues>(
            commands_topic_, rclcpp::SystemDefaultsQoS());

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TofGpioParamNode::OnParamChange, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "tof_gpio_param_node ready. gpio_group='%s' -> topic='%s'",
                    gpio_group_.c_str(), commands_topic_.c_str());
    }

private:
    // Converts a parameter's value to the double the wire message needs,
    // or std::nullopt if the type isn't one we know how to forward.
    static std::optional<double> ParamToDouble(const rclcpp::Parameter &p)
    {
        switch (p.get_type())
        {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return p.as_bool() ? 1.0 : 0.0;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return static_cast<double>(p.as_int());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return p.as_double();
        default:
            return std::nullopt;
        }
    }

    void OnHardwareState(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
    {
        for (size_t g = 0; g < msg->interface_groups.size(); g++)
        {
            if (msg->interface_groups[g] != gpio_group_)
                continue;

            const auto &iv = msg->interface_values[g];
            for (size_t i = 0; i < iv.interface_names.size(); i++)
            {
                const auto &name = iv.interface_names[i];
                if (!this->has_parameter(name))
                    continue;

                // Only sync fields you actually want mirrored back --
                // needs_calibration is the main one that self-clears.
                if (name != "needs_calibration")
                    continue;

                syncing_from_hw_ = true;
                this->set_parameter(rclcpp::Parameter(name, iv.values[i] != 0.0));
                syncing_from_hw_ = false;
            }
        }
    }

    // at the top of OnParamChange, before the loop:

    rcl_interfaces::msg::SetParametersResult OnParamChange(
        const std::vector<rclcpp::Parameter> &params)
    {
        if (syncing_from_hw_)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result; // don't re-publish updates that came FROM the hardware
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        std::vector<std::string> changed_names;
        std::vector<double> changed_values;

        for (const auto &p : params)
        {
            // Node's own config -- reconfigures this node, not the hardware.
            if (reserved_params_.count(p.get_name()))
            {
                continue;
            }

            auto value = ParamToDouble(p);
            if (!value.has_value())
            {
                result.successful = false;
                result.reason = p.get_name() + ": unsupported parameter type "
                                               "(only bool/int/double are forwarded to gpio_controller)";
                return result;
            }

            changed_names.push_back(p.get_name());
            changed_values.push_back(*value);
        }

        if (!changed_names.empty())
        {
            PublishChanges(changed_names, changed_values);
        }

        return result;
    }

    void PublishChanges(
        const std::vector<std::string> &names,
        const std::vector<double> &values)
    {
        DynamicInterfaceGroupValues msg;
        msg.interface_groups = {gpio_group_};

        InterfaceValue iv;
        iv.interface_names = names;
        iv.values = values;
        msg.interface_values = {iv};

        publisher_->publish(msg);

        std::string logline;
        for (size_t i = 0; i < names.size(); i++)
        {
            logline += names[i] + "=" + std::to_string(values[i]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Published to %s [%s]: %s",
                    gpio_group_.c_str(), commands_topic_.c_str(), logline.c_str());
    }

    std::string gpio_group_;
    std::string commands_topic_;
    std::set<std::string> reserved_params_;
    rclcpp::Publisher<DynamicInterfaceGroupValues>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    // add near the other members
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr state_sub_;
    bool syncing_from_hw_ = false; // true while we're applying an HW-driven update
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TofGpioParamNode>());
    rclcpp::shutdown();
    return 0;
}