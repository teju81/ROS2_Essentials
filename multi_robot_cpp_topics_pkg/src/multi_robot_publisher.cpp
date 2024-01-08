// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "multi_robot_interfaces_pkg/msg/address_book.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MultiRobotCppPublisherClass : public rclcpp::Node
{
public:
  MultiRobotCppPublisherClass()
  : Node("multi_robot_cpp_publisher_node"), count_(0), std_queue_size_(10), custom_queue_size_(10)
  {
    // Publisher and timer for the std msgs Interface
    publisher_std_ = this->create_publisher<std_msgs::msg::String>("multi_robot_std_topic", std_queue_size_);
    timer_std_ = this->create_wall_timer(
      500ms, std::bind(&MultiRobotCppPublisherClass::timer_std_callback, this));

    // Publisher and timer for the custom msgs Interface
    publisher_custom_ = this->create_publisher<multi_robot_interfaces_pkg::msg::AddressBook>("multi_robot_custom_topic", custom_queue_size_);
    timer_custom_ = this->create_wall_timer(
      200ms, std::bind(&MultiRobotCppPublisherClass::timer_custom_callback, this));
  }

private:
  void timer_std_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing std msg: '%s'", message.data.c_str());
    publisher_std_->publish(message);
  }

    void timer_custom_callback()
  {
    auto message = multi_robot_interfaces_pkg::msg::AddressBook();
    message.first_name = "John";
    message.last_name = "Doe";
    message.phone_number = "1234567890";
    message.phone_type = message.PHONE_TYPE_MOBILE;

    std::cout << "Publishing Contact\nFirst: " << message.first_name << " Last: " << message.last_name << std::endl;


    RCLCPP_INFO(this->get_logger(), "Publishing custom msg: '%s'", message.first_name.c_str());
    publisher_custom_->publish(message);
  }


  rclcpp::TimerBase::SharedPtr timer_std_, timer_custom_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_std_;
  rclcpp::Publisher<multi_robot_interfaces_pkg::msg::AddressBook>::SharedPtr publisher_custom_;

  size_t count_, custom_queue_size_, std_queue_size_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotCppPublisherClass>());
  rclcpp::shutdown();
  return 0;
}