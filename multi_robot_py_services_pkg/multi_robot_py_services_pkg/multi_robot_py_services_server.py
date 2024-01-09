from multi_robot_interfaces_pkg.srv import PhoneNumberBelongsToService

import rclpy
from rclpy.node import Node


class PhoneNumberBelongsToServiceServerClass(Node):

    def __init__(self):
        super().__init__('phone_number_belongs_to_service_server')
        self.srv = self.create_service(PhoneNumberBelongsToService, 'phone_number_belongs_to_service', self.phone_number_belongs_to_service_server_callback)

    def phone_number_belongs_to_service_server_callback(self, request, response):
        response.details.first_name = "John"
        response.details.last_name = "Doe"
        self.get_logger().info('Incoming request\nphone number: %s' % (request.phone_number))
        self.get_logger().info('phone number: %s belongs to First Name: %s Last Name: %s' % (request.phone_number, response.details.first_name, response.details.last_name))

        return response


def main():
    rclpy.init()

    phone_number_belongs_to_service_obj = PhoneNumberBelongsToServiceServerClass()

    rclpy.spin(phone_number_belongs_to_service_obj)

    rclpy.shutdown()


if __name__ == '__main__':
    main()