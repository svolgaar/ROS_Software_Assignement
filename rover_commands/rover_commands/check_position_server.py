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
        self.get_logger().info(f'Checking position: x={x}, z={z}')
        
        # Check if the position is within the boundaries
        if x < -100 or x > 100 or z < -100 or z > 100:
            response.is_allowed = False
            response.suggestion = 'Move within the boundaries'
        else:
            response.is_allowed = True
            response.suggestion = 'No suggestion'

        self.get_logger().info(f'Received request: x={x}, z={z}')
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