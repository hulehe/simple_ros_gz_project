#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>
#include <limits>

class ObstacleAvoider : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::string robot_name;
    double lidar_offset;

public:
    ObstacleAvoider() : Node("obstacle_avoider")
    {
        this->declare_parameter("robot_name", "diff_drive_robot");
        this->get_parameter("robot_name", robot_name);
        std::string cmd_vel_topic = "/" + robot_name + "/cmd_vel";
        std::string scan_topic = "/" + robot_name + "/scan";

        this->declare_parameter("lidar_offset", 1.0);
        this->get_parameter("lidar_offset", lidar_offset);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic,
            10,
            std::bind(&ObstacleAvoider::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoider Node Started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const auto &ranges = msg->ranges;
        int total_readings = ranges.size();
        int center_index = total_readings / 2;
        int window = 15; // 中心 ±15 个点

        float min_distance = std::numeric_limits<float>::infinity();

        int start = std::max(0, center_index - window);
        int end = std::min(total_readings, center_index + window);

        for (int i = start; i < end; ++i)
        {
            float r = ranges[i];
            if (std::isfinite(r))
            {
                min_distance = std::min(min_distance, r);
            }
        }

        auto twist_msg = geometry_msgs::msg::Twist();

        RCLCPP_DEBUG(this->get_logger(), "min_distance = %.3f", min_distance);

        if (min_distance < lidar_offset + 0.8)
        {
            // 距离太近，后退并转弯
            twist_msg.linear.x = -0.1;
            twist_msg.angular.z = 0.5;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m. Avoiding...", min_distance);
        }
        else
        {
            // 正常前进
            twist_msg.linear.x = 0.2;
            twist_msg.angular.z = 0.0;
        }

        cmd_pub_->publish(twist_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}
