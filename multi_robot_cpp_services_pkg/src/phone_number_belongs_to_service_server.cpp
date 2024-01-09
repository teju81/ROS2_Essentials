#include<string>

#include "rclcpp/rclcpp.hpp"
#include "multi_robot_interfaces_pkg/srv/phone_number_belongs_to_service.hpp"

#include <memory>

void phone_number_belongs_to_service(const std::shared_ptr<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService::Request> request,
          std::shared_ptr<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService::Response>      response)
{
    std::string first_name, last_name, phone_number;
    first_name = "Unknown";
    last_name = "Unknown";
    phone_number = "1234567890";

    response->details.first_name = "John";
    response->details.last_name = "Doe";

    /*
    if (strcmp(request->phone_number,phone_number.c_str()))
    {
      response->details.first_name = first_name.c_str();
      response->details.last_name = last_name.c_str();
    }
    */

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request \nphone number: %s",
                request->phone_number.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: First Name [%s], Last Name [%s]", response->details.first_name.c_str(), response->details.last_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("phone_number_belongs_to_service_server");

  rclcpp::Service<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService>::SharedPtr service =
    node->create_service<multi_robot_interfaces_pkg::srv::PhoneNumberBelongsToService>("phone_number_belongs_to_service", &phone_number_belongs_to_service);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to furnish details against phone number.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}