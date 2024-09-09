import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.srv import CheckPosition

class TrajectorySubscriber(Node):

        def __init__(self):
            super().__init__('trajectory_subscriber')
            # TODO: Create a subscriber of type Twist, that calls listener_callback
            # Your code here
            self.subscription = self.create_subscription(
                    Twist, 
                    'rover/cmd_vel', 
                    self.listener_callback, 
                    10)
            self.cli = self.create_client(CheckPosition, 'check_position')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
            self.req = CheckPosition.Request()
            self.get_logger().info('Subscriber node has been started.')
            self.position = {'x': 0.0, 'z': 0.0, 'ry': 0.0}
       
        def send_request(self, x, z):
            self.req.x = x
            self.req.z = z
            self.get_logger().info(f'Sending request: x={x}, z={z}')
            
            # Call the service asynchronously
            self.future = self.cli.call_async(self.req)
            self.get_logger().info('Service call sent')

            # Add a callback to the future that will be called when the future is done
            self.future.add_done_callback(self.handle_service_response)

        def handle_service_response(self, future):
            if future.done() and future.exception() is None:
                response = future.result()
                self.get_logger().info(f'Service response: is_allowed={response.is_allowed}, suggestion={response.suggestion}')
                # Process the response here
            else:
                self.get_logger().error('Service call failed')
        def listener_callback(self, msg):
           # TODO: Interpret the received commands and log the result using self.get_logger().info()
           # Your code here
            movement_map = {
                (1.0, 0.0, 0.0): "Go Forward",
                (-1.0, 0.0, 0.0): "Go Backward",
                (0.0, 1.0, 0.0): "Slide Left",
                (0.0, -1.0, 0.0): "Slide Right",
                (0.0, 0.0, 1.0): "Rotating on itself to the Left",
                (0.0, 0.0, -1.0): "Rotating on itself to the Right",
                (1.0,0.0,1.0): "Go Forward and Rotate to the Left",
                (1.0,0.0,-1.0): "Go Forward and Rotate to the Right",
                (-1.0,0.0,1.0): "Go Backward and Rotate to the Left",
                (-1.0,0.0,-1.0): "Go Backward and Rotate to the Right",
                #Forbidden
                #Tx and Tz
                (1.0, 1.0, 0.0): "Forbidden",
                (1.0, -1.0, 0.0): "Forbidden",
                (-1.0, 1.0, 0.0): "Forbidden",
                (-1.0, -1.0, 0.0): "Forbidden",
                #Tz and Ry
                (0.0, 1.0, 1.0): "Forbidden",
                (0.0, 1.0, -1.0): "Forbidden",
                (0.0, -1.0, 1.0): "Forbidden",
                (0.0, -1.0, -1.0): "Forbidden",
            }
            x = msg.linear.x
            z = msg.linear.z
            ry = msg.angular.y

            command = (x, z, ry)
            self.get_logger().info(f'Command received: {command}')
            # Check if the position is within the boundarie
            response = self.send_request(self.position['x'] + x, self.position['z'] + z)
            if response is None:
                self.get_logger().error("Failed to receive a valid response from the service.")
                return 
            if response.is_allowed:
                movement = movement_map.get(command, "Unknown command")
                self.get_logger().info(f'Command received: {movement}')
                # Update position
                self.position['x'] += command[0]
                self.position['z'] += command[1]
                self.position['ry'] += command[2]
                self.get_logger().info(f'New Position: {self.position}')

            else:  
                self.get_logger().info(f'Command received: Forbidden')
                self.get_logger().info(f'New Position: {self.position}')
def main(args=None):
       rclpy.init(args=args)
       node = TrajectorySubscriber()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
    main()