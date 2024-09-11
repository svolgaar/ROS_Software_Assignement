import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.srv import CheckPosition
import sys

class Client(Node):

    def __init__(self):
        super().__init__('client')
        # TODO: Create a server client of type CheckPosition
        # Your code here
        self.cli = self.create_client(CheckPosition, 'check_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CheckPosition.Request()
        self.get_logger().info('Client node has been started.')
        self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}

    def send_request(self, x, z):
        self.req.x = x
        self.req.z = z
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Before spin_until_future_complete, future done: {self.future.done()}')
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f'After spin_until_future_complete, future result: {self.future.result()}')
        return self.future.result()
            # TODO: Send the request to the server synchronously
            # Your code here

def main(args=None):
    rclpy.init(args=args)
    client = Client()
    response = client.send_request(float(sys.argv[1]), float(sys.argv[2]))
    rclpy.spin(client)
    client.get_logger().info(
        f'Response: is_allowed={response.is_allowed}, suggestion="{response.suggestion}"'
        )
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()