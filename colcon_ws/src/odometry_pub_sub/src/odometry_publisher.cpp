#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
  public:
    OdometryPublisher() : Node("odometry_publisher")
    {
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("vehicle/odometry", 50);
      timer_ = this->create_wall_timer(20ms, std::bind(&OdometryPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto odom_data = nav_msgs::msg::Odometry();

      odom_data.header.stamp = rclcpp::Clock().now();
      odom_data.header.frame_id = "map";

      odom_data.child_frame_id = "odom";

      odom_data.pose.pose.position.x = 4.0;
      odom_data.pose.pose.position.y = 4.0;
      odom_data.pose.pose.position.z = 0.0;
      odom_data.pose.pose.orientation.x = 0.0;
      odom_data.pose.pose.orientation.y = 0.0;
      odom_data.pose.pose.orientation.z = 0.0;
      odom_data.pose.pose.orientation.w = 1.0;

      odom_data.twist.twist.linear.x = 0.0;
      odom_data.twist.twist.linear.y = 0.0;
      odom_data.twist.twist.linear.z = 0.0;
      odom_data.twist.twist.angular.x = 0.0;
      odom_data.twist.twist.angular.y = 0.0;
      odom_data.twist.twist.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(), "Publishing odometry message!");
      
      odom_publisher_->publish(odom_data);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}