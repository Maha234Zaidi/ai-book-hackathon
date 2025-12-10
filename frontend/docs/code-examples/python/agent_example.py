import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ComplexRobotAgent(Node):
    """
    A complex robot agent that demonstrates multiple ROS 2 concepts including:
    - Multiple subscribers for different sensor inputs
    - Publishers for robot control
    - State management
    - Complex behaviors based on sensor data
    """
    
    def __init__(self):
        super().__init__('complex_robot_agent')
        
        # State management
        self.robot_state = 'SEARCHING'  # SEARCHING, AVOIDING, GOAL_REACHED
        self.obstacle_detected = False
        self.goal_reached = False
        self.safety_distance = 0.5  # meters
        self.linear_velocity = 0.5  # default forward speed
        
        # Create publishers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'agent_status', 10)
        
        # Create subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Log initialization
        self.get_logger().info('Complex Robot Agent initialized')
    
    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Find minimum distance in forward arc
        min_distance = float('inf')
        forward_start = len(msg.ranges) // 2 - 30  # Approximate forward direction
        forward_end = len(msg.ranges) // 2 + 30
        
        for i in range(forward_start, forward_end):
            i = i % len(msg.ranges)  # Handle wrap-around
            if not math.isnan(msg.ranges[i]) and msg.ranges[i] < min_distance:
                min_distance = msg.ranges[i]
        
        # Update obstacle detection
        self.obstacle_detected = min_distance < self.safety_distance
        
        # Update state based on sensor input
        if self.obstacle_detected:
            self.robot_state = 'AVOIDING'
        else:
            self.robot_state = 'SEARCHING'
    
    def control_loop(self):
        """Main control loop that manages robot behavior based on state"""
        cmd_msg = Twist()
        
        if self.robot_state == 'SEARCHING':
            # Move forward when path is clear
            cmd_msg.linear.x = self.linear_velocity
            cmd_msg.angular.z = 0.0
            self.get_logger().debug('State: SEARCHING - Moving forward')
            
        elif self.robot_state == 'AVOIDING':
            # Implement simple obstacle avoidance behavior
            cmd_msg.linear.x = 0.0  # Stop forward motion
            cmd_msg.angular.z = 0.5  # Turn counterclockwise
            self.get_logger().debug('State: AVOIDING - Turning to avoid obstacle')
        
        # Publish command
        self.cmd_publisher.publish(cmd_msg)
        
        # Publish agent status
        status_msg = String()
        status_msg.data = f'State: {self.robot_state}, Obstacle: {self.obstacle_detected}'
        self.status_publisher.publish(status_msg)
    
    def change_behavior(self, new_behavior):
        """Method to externally change robot behavior"""
        old_state = self.robot_state
        self.robot_state = new_behavior
        self.get_logger().info(f'Behavior changed from {old_state} to {new_behavior}')


def main(args=None):
    """Main function to run the complex robot agent"""
    rclpy.init(args=args)
    
    agent = ComplexRobotAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.get_logger().info('Shutting down Complex Robot Agent')
        # Stop the robot before shutting down
        stop_msg = Twist()
        agent.cmd_publisher.publish(stop_msg)
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()