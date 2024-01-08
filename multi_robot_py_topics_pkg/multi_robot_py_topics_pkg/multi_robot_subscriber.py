import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from multi_robot_interfaces_pkg.msg import AddressBook


class MultiRobotPySubscriberClass(Node):

    def __init__(self):
        super().__init__('multi_robot_subscriber_node')
        self.std_queue_size_ = 10
        self.subscription_std_ = self.create_subscription(
            String,
            'multi_robot_std_topic',
            self.std_listener_callback,
            self.std_queue_size_)
        self.subscription_std_  # prevent unused variable warning

        self.custom_queue_size_ = 10
        self.subscription_custom_ = self.create_subscription(
            AddressBook,
            'multi_robot_custom_topic',
            self.custom_listener_callback,
            self.custom_queue_size_)
        self.subscription_custom_  # prevent unused variable warning

    def std_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


    def custom_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.first_name)

def main(args=None):
    rclpy.init(args=args)

    multi_robot_py_subscriber_obj = MultiRobotPySubscriberClass()

    rclpy.spin(multi_robot_py_subscriber_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_robot_py_subscriber_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()