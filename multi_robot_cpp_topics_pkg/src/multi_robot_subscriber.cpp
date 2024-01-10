#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "multi_robot_interfaces_pkg/msg/address_book.hpp"

using std::placeholders::_1;

class MultiRobotCppSubscriber : public rclcpp::Node
{
  public:
    MultiRobotCppSubscriber()
    : Node("multi_robot_subscriber_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)), std_queue_size_(10), custom_queue_size_(10)
    {

    std::string std_topic_name = this->get_parameter("std_topic_name").as_string();

    std::string custom_topic_name = this->get_parameter("custom_topic_name").as_string();


      // Create two subscribers, one each for multi_robot_std_topic and multi_robot_custom_topic topics
      std_subscription_ = this->create_subscription<std_msgs::msg::String>(
      std_topic_name, std_queue_size_, std::bind(&MultiRobotCppSubscriber::std_topic_callback, this, _1));

      custom_subscription_ = this->create_subscription<multi_robot_interfaces_pkg::msg::AddressBook>(
      custom_topic_name, custom_queue_size_, std::bind(&MultiRobotCppSubscriber::custom_topic_callback, this, _1));
    }

  private:
    // callback function for the multi_robot_std_topic subscriber
    void std_topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    // callback function for the multi_robot_custom_topic subscriber
    void custom_topic_callback(const multi_robot_interfaces_pkg::msg::AddressBook & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.first_name.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr std_subscription_;
    rclcpp::Subscription<multi_robot_interfaces_pkg::msg::AddressBook>::SharedPtr custom_subscription_;
    size_t std_queue_size_, custom_queue_size_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotCppSubscriber>());
  rclcpp::shutdown();
  return 0;
}