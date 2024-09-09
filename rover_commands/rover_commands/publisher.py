import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        # TODO: Create a publisher of type Twist
        # Your code here
        self.publisher_ = self.create_publisher(Twist, 'rover/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.cmd_acquisition)
        self.get_logger().info('Publisher node has been started.')

        # TODO: Create a loop here to ask users a prompt and send messages accordingly


    # Function that prompts user for a direction input, and sends the command
    def cmd_acquisition(self):
        command = input("Enter command (w/a/s/d/t/y - max 2 characters): ")
        # TODO: Complete the function to transform the input into the right command.
        # Your code here
        twist = Twist() 
        if command == 'w':
            twist.linear.x = 1.0
        elif command == 's':
            twist.linear.x = -1.0
        elif command == 'a':
            twist.linear.z = 1.0
        elif command == 'd':
            twist.linear.z = -1.0
        elif command == 't':
            twist.angular.y = 1.0
        elif command == 'y':
            twist.angular.y = -1.0

        self.publisher_.publish(twist)
        self.get_logger().info(f'Published command: {command}')

        pass

def main(args=None):
    rclpy.init(args=args)   # Init ROS python
    node = TrajectoryPublisher()  # Create a Node instance
    rclpy.spin(node)  # Run the node in a Thread
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
