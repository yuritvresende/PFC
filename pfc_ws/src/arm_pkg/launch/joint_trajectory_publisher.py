import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'arm_controller/command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Node started. Enter joint positions separated by spaces.')

    def timer_callback(self):
        try:
            input_str = input("Enter joint positions (separated by spaces): ")
            positions = list(map(float, input_str.split()))
        except ValueError:
            self.get_logger().error('Invalid input. Please enter float values separated by spaces.')
            return

        # Create and populate JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # Adjust joint names as per your robot

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1  # Time to reach the target position
        msg.points.append(point)

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint positions: {positions}')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
