import rclpy
from rclpy.node import Node
from custom_msgs.srv import CheckPosition

class CheckPositionServer(Node):

    def __init__(self):
        super().__init__('check_position_server')
        # TODO: Create a server of type CheckPosition.srv that calls check_position_callback at each request.
        # Your code here
        self.srv = self.create_service(CheckPosition, 'check_position', self.check_position_callback)
        self.get_logger().info('Service server has been started.')

    def check_position_callback(self, request, response):
        # TODO: Get the inputs from the request, and process them to check the boundaries
        # Your code here
        x = request.x
        z = request.z
        ry = request.ry
        self.get_logger().info(f'Received request: x={x}, z={z}')
        boundary_checks = {
            'z': [
                (lambda val: val < -100.0, 'Z position out of bounds, move right'),
                (lambda val: val > 100.0, 'Z position out of bounds, move left'),
                (lambda val: val < -95.0, 'Z position close to the left boundary, consider moving right'),
                (lambda val: val > 95.0, 'Z position close to the right boundary, consider moving left'),

            ],
            'x': [
                (lambda val: val < -100.0, 'X position out of bounds, move forward'),
                (lambda val: val > 100.0, 'X position out of bounds, move backward'),
                (lambda val: val < -95.0, 'X position close to the forward boundary, consider moving backward'),
                (lambda val: val > 95.0, 'X position close to the backward boundary, consider moving forward'),
            ]

        }
        for axis, axis_val in [('x', x), ('z', z)]:
            for check, suggestion in boundary_checks[axis]:
                if check(axis_val):
                    response.is_allowed = False
                    response.suggestion = suggestion
                    return response
            else:
                continue
        else:
            response.is_allowed = True
            response.suggestion = 'Position is allowed'
        self.get_logger().info(f'Response: is_allowed={response.is_allowed}, suggestion="{response.suggestion}"')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CheckPositionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()