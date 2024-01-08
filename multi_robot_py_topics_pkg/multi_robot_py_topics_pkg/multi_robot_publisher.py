import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from multi_robot_interfaces_pkg.msg import AddressBook



class MultiRobotPyPublisherClass(Node):

    def __init__(self):
        super().__init__('multi_robot_py_publisher_node')
        self.std_queue_size_ = 10
        self.publisher_std_ = self.create_publisher(String, 'multi_robot_std_topic', self.std_queue_size_)
        std_timer_period = 0.5  # seconds
        self.std_timer = self.create_timer(std_timer_period, self.std_timer_callback)
        self.i = 0

        self.custom_queue_size_ = 10
        self.publisher_custom_ = self.create_publisher(AddressBook, 'multi_robot_custom_topic', self.custom_queue_size_)
        custom_timer_period = 0.5  # seconds
        self.custom_timer = self.create_timer(custom_timer_period, self.custom_timer_callback)
        self.i = 0

    def std_timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_std_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def custom_timer_callback(self):
        msg = AddressBook()
        msg.first_name = 'John'
        msg.last_name = 'John'
        msg.phone_number = '1234567890'
        msg.phone_type = msg.PHONE_TYPE_MOBILE;
        self.publisher_custom_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.first_name)


def main(args=None):
    rclpy.init(args=args)

    multi_robot_py_publisher_obj = MultiRobotPyPublisherClass()

    rclpy.spin(multi_robot_py_publisher_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_robot_py_publisher_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()