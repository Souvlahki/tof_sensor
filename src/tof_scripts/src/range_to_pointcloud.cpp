#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class TofToPointCloud : public rclcpp::Node
{
public:
    TofToPointCloud() : Node("tof_to_pointcloud_converter")
    {
        // Subscribe to the Range topic published by ros2_control
        range_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/range_sensor_broadcaster/range", 10,
            std::bind(&TofToPointCloud::range_callback, this, std::placeholders::_1));

        // Publish the PointCloud2 for Nav2 local_costmap
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tof_cloud_1", 10);

        RCLCPP_INFO(this->get_logger(), "ToF Range-to-PointCloud converter initialized.");
        RCLCPP_INFO(this->get_logger(), "HELLO");
    }

private:
    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        // Ignore out-of-range noise
        if (msg->range < msg->min_range || msg->range > msg->max_range)
        {
            return;
        }

        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        cloud_msg->header.stamp = this->now();

        cloud_msg->header.frame_id = "tof_sensor_1_link";

        cloud_msg->height = 1;
        cloud_msg->width = 1;
        cloud_msg->is_dense = true;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(1);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        *iter_x = static_cast<float>(msg->range);
        *iter_y = 0.0f;
        *iter_z = 0.0f;

        pointcloud_pub_->publish(*cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TofToPointCloud>());
    rclcpp::shutdown();
    return 0;
}