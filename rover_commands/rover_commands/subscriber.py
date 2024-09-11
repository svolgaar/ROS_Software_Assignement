import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.srv import CheckPosition
import threading
class TrajectorySubscriber(Node):

        def __init__(self):
            super().__init__('trajectory_subscriber')
            # TODO: Create a subscriber of type Twist, that calls listener_callback
            # Your code here
            self.subscription = self.create_subscription(Twist, 'commands', self.listener_callback, 10)
            self.cli = self.create_client(CheckPosition, 'check_position')
            self.call_service_flag = False
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = CheckPosition.Request()
            self.get_logger().info('Subscriber node has been started.')
            self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}
       
        def send_request(self,x,z,ry):
            self.req.x = x
            self.req.z = z
            self.future = self.cli.call_async(self.req)
            self.get_logger().info(f'Before spin_until_future_complete, future done: {self.future.done()}')
            rclpy.spin_until_future_complete(self, self.future)
            self.get_logger().info(f'After spin_until_future_complete, future result: {self.future.result()}')
            return self.future.result()

        def listener_callback(self, msg):
           # TODO: Interpret the received commands and log the result using self.get_logger().info()
           # Your code here
            linear_speed = msg.linear.x
            angular_speed = msg.angular.z
            movement_map = {
                'Tx': ('Go Forward', 'Go Backward', 'x', msg.linear.x),
                'Ry': ('Rotating on itself to the Left', 'Rotating on itself to the Right', 'ry', msg.angular.z),
                'Tz': ('Slide Left', 'Slide Right', 'z', msg.linear.z),
                # Add more movements if needed
            }
            if (msg.linear.x != 0 and msg.linear.z != 0) or (msg.angular.z != 0 and msg.linear.z != 0):
                self.get_logger().info('Forbidden move')
            else: 
                for movement, (positive_action, negative_action, axis, speed) in movement_map.items():
                    if speed > 0:
                        self.get_logger().info(positive_action)
                        self.position[axis] += speed
                    elif speed < 0:
                        self.get_logger().info(negative_action)
                        self.position[axis] += speed      
                self.get_logger().info(f'New Position: {self.position}')
                self.call_service_flag = True

        def call_service(self):
            # Send the request to the service with the updated position
            response = self.send_request(float(self.position['x']), float(self.position['z']), float(self.position['ry']))
            if response.is_allowed is not None:
                self.get_logger().info(f'Position is allowed: {response.is_allowed}, suggestion: {response.suggestion}')
            # Log the new position
            self.get_logger().info(f'New Position: {self.position}')
            self.call_service_flag = False

def main(args=None):
        rclpy.init(args=args)
        node = TrajectorySubscriber()
        try:
            while rclpy.ok():
                rclpy.spin_once(node)
                if node.call_service_flag:
                    node.call_service()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()