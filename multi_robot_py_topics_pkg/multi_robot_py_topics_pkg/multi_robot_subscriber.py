import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from multi_robot_interfaces_pkg.msg import AddressBook
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

class MultiRobotPyImageSubscriberClass(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('multi_robot_image_subscriber_node')
          
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          Image, 
          'video_frames', 
          self.listener_callback, 
          10)
        self.subscription # prevent unused variable warning
          
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
   
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
     
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data,"bgr8")
        
        # Display image
        cv2.imshow("camera", current_frame)
        
        cv2.waitKey(1)

class MultiRobotPyStringSubscriberClass(Node):

    def __init__(self):
        super().__init__('multi_robot_string_subscriber_node')
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

    multi_robot_py_image_subscriber_obj = MultiRobotPyImageSubscriberClass()

    rclpy.spin(multi_robot_py_image_subscriber_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_robot_py_image_subscriber_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()