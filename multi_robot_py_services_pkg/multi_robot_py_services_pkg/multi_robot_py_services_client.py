import sys

from multi_robot_interfaces_pkg.srv import PhoneNumberBelongsToService
import rclpy
from rclpy.node import Node


class PhoneNumberBelongsToServiceClientAsyncClass(Node):

    def __init__(self):
        super().__init__('phone_number_belongs_to_service_client_async')
        self.cli = self.create_client(PhoneNumberBelongsToService, 'phone_number_belongs_to_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PhoneNumberBelongsToService.Request()

    def send_request(self, phone_number):
        self.req.phone_number = phone_number
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    phone_number_belongs_to_service_obj = PhoneNumberBelongsToServiceClientAsyncClass()
    response = phone_number_belongs_to_service_obj.send_request(sys.argv[1])
    print(sys.argv[1])
    phone_number_belongs_to_service_obj.get_logger().info(
        'Phone Number belongs to: First Name: %s Last Name: %s' % (response.details.first_name, response.details.last_name))

    phone_number_belongs_to_service_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()