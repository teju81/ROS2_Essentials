#include<string>

#include "rclcpp/rclcpp.hpp"
#include "multi_robot_interfaces_pkg/srv/phone_number_belongs_to_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "argv[1]: %s", argv[1]);

  /*
  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: phone_number_belongs_to_service_client phone_number");
      return 1;
  }
  */

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("phone_number_belongs_to_service_client");
  rclcpp::Client<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService>::SharedPtr client =
    node->create_client<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService>("phone_number_belongs_to_service");

  auto request = std::make_shared<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService::Request>();
  request->phone_number = argv[1];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto msg = result.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "First Name: %s", msg->details.first_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Last Name: %s", msg->details.last_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service phone_number_belongs_to_service");
  }

  rclcpp::shutdown();
  return 0;
}