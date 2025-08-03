#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    if(argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "you must give 2 seperate int numbers!");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_client");

    // create service client
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr service_client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_numbers");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atol(argv[1]);
    request->b = atol(argv[2]);

    while(!service_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto result = service_client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum = %ld", result.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_to_numbers.");
    }

    rclcpp::shutdown();

    return 0;
}