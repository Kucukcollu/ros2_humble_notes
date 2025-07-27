#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class OdometrySubscriber : public rclcpp::Node
{
  public:
    OdometrySubscriber() : Node("odometry_subscriber")
    {
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/odometry", 50, std::bind(&OdometrySubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------");

      RCLCPP_INFO(this->get_logger(), "Subscribed odometry message!");

      RCLCPP_INFO(this->get_logger(), "Position -> x: %.2f, y: %.2f, z: %.2f",
                  msg.pose.pose.position.x,
                  msg.pose.pose.position.y,
                  msg.pose.pose.position.z);

      RCLCPP_INFO(this->get_logger(), "Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
                  msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w);

      RCLCPP_INFO(this->get_logger(), "Linear Velocity -> x: %.2f, y: %.2f, z: %.2f",
                  msg.twist.twist.linear.x,
                  msg.twist.twist.linear.y,
                  msg.twist.twist.linear.z);

      RCLCPP_INFO(this->get_logger(), "Angular Velocity -> x: %.2f, y: %.2f, z: %.2f",
                  msg.twist.twist.angular.x,
                  msg.twist.twist.angular.y,
                  msg.twist.twist.angular.z);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySubscriber>());
  rclcpp::shutdown();
  return 0;
}