# Chapter 2: Integrating Python Agents with ROS 2 via rclpy

## Introduction to Python Agent Integration

In this chapter, we'll explore how to integrate Python-based AI agents with ROS 2 systems using the rclpy client library. This integration enables AI agents to interact with robotic hardware and sensor data, creating intelligent robotic systems that can perceive, reason, and act in the physical world.

Python agents in robotics typically perform tasks such as:
- Processing sensor data for perception
- Planning robot movements and actions
- Making decisions based on environmental conditions
- Learning from experience to improve performance

## Understanding rclpy

**rclpy** is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl) and the ROS 2 communication libraries. This allows Python programs to interact with ROS 2 systems using the same messaging and communication patterns as other ROS 2 nodes.

Key features of rclpy:
- Object-oriented interface for ROS 2 concepts
- Support for all ROS 2 communication patterns (topics, services, actions)
- Integration with Python's asyncio for asynchronous operations
- Memory management and type safety

## Creating AI Agents as ROS 2 Nodes

### Basic AI Agent Structure

Let's create a simple AI agent that acts as a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import random

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Publishers for agent outputs
        self.decision_publisher = self.create_publisher(String, 'agent_decision', 10)
        self.velocity_publisher = self.create_publisher(Float32, 'agent_velocity', 10)

        # Subscribers for sensor data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        # Timer for agent decision-making cycle
        self.timer = self.create_timer(0.1, self.decision_callback)

        self.get_logger().info('Simple AI Agent initialized')

    def laser_callback(self, msg):
        """Process laser scan data from robot sensors"""
        # Process the laser scan to detect obstacles
        min_distance = min(msg.ranges) if msg.ranges else float('inf')
        self.get_logger().info(f'Minimum obstacle distance: {min_distance:.2f}m')

    def decision_callback(self):
        """Make decisions based on sensor data"""
        # Simple decision-making logic
        decision = self.make_simple_decision()

        # Publish decision
        msg = String()
        msg.data = decision
        self.decision_publisher.publish(msg)

        # Publish velocity command
        velocity_msg = Float32()
        velocity_msg.data = 0.5 if decision == 'FORWARD' else 0.0
        self.velocity_publisher.publish(velocity_msg)

        self.get_logger().info(f'Agent decision: {decision}')

    def make_simple_decision(self):
        """Simple decision-making logic"""
        # In a real agent, this would be replaced with more sophisticated logic
        # such as machine learning models or complex reasoning algorithms
        decisions = ['FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP']
        return random.choice(decisions)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Agent Patterns

### Stateful Agent with Memory

For more complex AI agents, you may need to maintain state between decisions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class StatefulAIAgent(Node):
    def __init__(self):
        super().__init__('stateful_ai_agent')

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, 'agent_state', 10)

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        # Agent state
        self.current_state = 'SEARCHING'
        self.last_decision_time = time.time()
        self.obstacle_detected = False
        self.movement_history = []

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_callback)

    def laser_callback(self, msg):
        """Process laser scan data"""
        if msg.ranges:
            min_distance = min(msg.ranges)
            self.obstacle_detected = min_distance < 1.0  # 1 meter threshold

    def decision_callback(self):
        """Make decisions based on current state and sensor data"""
        current_time = time.time()

        # Update state based on conditions
        if self.obstacle_detected and self.current_state != 'AVOIDING':
            self.current_state = 'AVOIDING'
            self.last_decision_time = current_time
        elif not self.obstacle_detected and self.current_state == 'AVOIDING':
            self.current_state = 'SEARCHING'
            self.last_decision_time = current_time

        # Execute appropriate behavior based on state
        cmd_vel = Twist()
        if self.current_state == 'AVOIDING':
            cmd_vel.angular.z = 0.5  # Turn to avoid obstacle
            cmd_vel.linear.x = 0.0
        else:
            cmd_vel.linear.x = 0.5   # Move forward
            cmd_vel.angular.z = 0.0

        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)

        # Publish state
        state_msg = String()
        state_msg.data = self.current_state
        self.state_publisher.publish(state_msg)

        self.get_logger().info(f'State: {self.current_state}, Obstacle: {self.obstacle_detected}')


def main(args=None):
    rclpy.init(args=args)
    ai_agent = StatefulAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        print('Stateful AI Agent stopped by user')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Complete Python Agent Example

Here's a complete Python agent implementation that demonstrates the concepts discussed:

```python
#!/usr/bin/env python3
"""
Simple Python Agent for ROS 2 Integration

This example demonstrates how to create a Python-based AI agent that integrates
with ROS 2 systems using the rclpy client library. The agent subscribes to sensor
data, processes it, and publishes control commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import random


class SimpleAIAgent(Node):
    """
    A simple AI agent that acts as a ROS 2 node.
    It subscribes to sensor data and publishes decisions and commands.
    """

    def __init__(self):
        super().__init__('simple_ai_agent')

        # Publishers for agent outputs
        self.decision_publisher = self.create_publisher(String, 'agent_decision', 10)
        self.velocity_publisher = self.create_publisher(Float32, 'agent_velocity', 10)

        # Subscribers for sensor data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        # Timer for agent decision-making cycle
        self.timer = self.create_timer(0.1, self.decision_callback)

        self.get_logger().info('Simple AI Agent initialized')

    def laser_callback(self, msg):
        """Process laser scan data from robot sensors"""
        # Process the laser scan to detect obstacles
        if msg.ranges:
            min_distance = min(msg.ranges)
            self.get_logger().info(f'Minimum obstacle distance: {min_distance:.2f}m')

    def decision_callback(self):
        """Make decisions based on sensor data"""
        # Simple decision-making logic
        decision = self.make_simple_decision()

        # Publish decision
        msg = String()
        msg.data = decision
        self.decision_publisher.publish(msg)

        # Publish velocity command
        velocity_msg = Float32()
        velocity_msg.data = 0.5 if decision == 'FORWARD' else 0.0
        self.velocity_publisher.publish(velocity_msg)

        self.get_logger().info(f'Agent decision: {decision}')

    def make_simple_decision(self):
        """
        Simple decision-making logic.

        In a real agent, this would be replaced with more sophisticated logic
        such as machine learning models or complex reasoning algorithms.
        """
        decisions = ['FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP']
        return random.choice(decisions)


def main(args=None):
    """Main function to run the AI agent node"""
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        print('Agent interrupted by user')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Create a Simple AI Agent
1. Create an AI agent that subscribes to sensor data
2. Implement basic decision-making logic
3. Publish commands based on the agent's decisions
4. Test the agent in a simulated environment

Example implementation:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import random

class SimpleDecisionAgent(Node):
    def __init__(self):
        super().__init__('simple_decision_agent')
        self.publisher_ = self.create_publisher(String, 'agent_output', 10)

        # Timer for decision making
        self.timer = self.create_timer(1.0, self.decision_callback)
        self.get_logger().info('Simple Decision Agent started')

    def decision_callback(self):
        """Make a simple decision and publish it"""
        decision_options = ['IDLE', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP']
        decision = random.choice(decision_options)

        msg = String()
        msg.data = decision
        self.publisher_.publish(msg)

        self.get_logger().info(f'Agent decision: {decision}')

def main(args=None):
    rclpy.init(args=args)
    agent = SimpleDecisionAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        print('Agent stopped by user')
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2: Implement Stateful Behavior
1. Create an AI agent that maintains state between decisions
2. Implement a finite state machine for the agent
3. Add transitions between states based on sensor inputs
4. Test state transitions with various input scenarios

### Exercise 3: Error Handling
1. Add comprehensive error handling to your AI agent
2. Implement fallback behaviors for different error conditions
3. Test error scenarios to ensure robust operation
4. Log errors and publish error notifications
```

### Sensor Data Processing

AI agents often need to process various types of sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Float32MultiArray
import numpy as np
from cv_bridge import CvBridge

class SensorProcessingAgent(Node):
    def __init__(self):
        super().__init__('sensor_processing_agent')

        # Publishers
        self.perception_publisher = self.create_publisher(Float32MultiArray, 'perception_output', 10)

        # Subscribers
        self.laser_subscriber = self.create_subscription(LaserScan, 'laser_scan', self.process_laser, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu_data', self.process_imu, 10)

        # CV Bridge for image processing
        self.cv_bridge = CvBridge()

        self.get_logger().info('Sensor Processing Agent initialized')

    def process_laser(self, msg):
        """Process laser scan data for obstacle detection"""
        if not msg.ranges:
            return

        # Calculate distance statistics
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if not valid_ranges:
            return

        avg_distance = sum(valid_ranges) / len(valid_ranges)
        min_distance = min(valid_ranges)

        # Create perception output
        perception_msg = Float32MultiArray()
        perception_msg.data = [min_distance, avg_distance, len(valid_ranges)]

        self.perception_publisher.publish(perception_msg)

    def process_imu(self, msg):
        """Process IMU data for orientation and acceleration"""
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        # Process IMU data for state estimation
        # (In a real implementation, this would integrate with a state estimator)

        self.get_logger().info(f'Orientation: ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})')
```

## Error Handling and Robustness

### Exception Handling in ROS 2 Nodes

AI agents should handle errors gracefully:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import traceback

class RobustAIAgent(Node):
    def __init__(self):
        super().__init__('robust_ai_agent')

        self.error_publisher = self.create_publisher(String, 'agent_errors', 10)
        self.main_publisher = self.create_publisher(String, 'agent_output', 10)

        # Timer with error handling
        self.timer = self.create_timer(0.1, self.safe_decision_callback)

    def safe_decision_callback(self):
        """Decision callback with error handling"""
        try:
            # Your AI logic here
            result = self.ai_decision_logic()

            # Publish result
            msg = String()
            msg.data = str(result)
            self.main_publisher.publish(msg)

        except Exception as e:
            # Log error
            error_msg = f'AI Agent Error: {str(e)}\n{traceback.format_exc()}'
            self.get_logger().error(error_msg)

            # Publish error notification
            error_pub_msg = String()
            error_pub_msg.data = str(e)
            self.error_publisher.publish(error_pub_msg)

            # Fallback behavior
            self.execute_fallback_behavior()

    def ai_decision_logic(self):
        """Main AI decision logic - may raise exceptions"""
        # Placeholder for actual AI logic
        # This could be a neural network inference, planning algorithm, etc.
        return "decision_result"

    def execute_fallback_behavior(self):
        """Execute safe fallback behavior when errors occur"""
        self.get_logger().info('Executing fallback behavior')
        # Implement safe behavior (e.g., stop robot, return to safe state)
```

## Integration Patterns

### Publisher-Subscriber Integration

The most common pattern for AI agent integration is using the publisher-subscriber model:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class ControlAgent(Node):
    def __init__(self):
        super().__init__('control_agent')

        # Publishers for control commands
        self.trajectory_publisher = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)

        # Subscribers for state feedback
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.controller_state_subscriber = self.create_subscription(
            JointTrajectoryControllerState, 'controller_state', self.controller_state_callback, 10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Internal state
        self.current_joint_positions = {}
        self.target_positions = {}

    def joint_state_callback(self, msg):
        """Update internal state with current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def controller_state_callback(self, msg):
        """Process controller state feedback"""
        # Update target positions from controller state if needed
        pass

    def control_loop(self):
        """Main control loop - makes decisions based on current state"""
        # Example: Simple position control
        if 'joint1' in self.current_joint_positions:
            current_pos = self.current_joint_positions['joint1']
            target_pos = np.sin(self.get_clock().now().nanoseconds * 1e-9)  # Oscillating target

            # Create trajectory command
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['joint1']

            point = JointTrajectoryPoint()
            point.positions = [target_pos]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 50000000  # 50ms

            trajectory_msg.points = [point]

            self.trajectory_publisher.publish(trajectory_msg)
```

## Exercises

### Exercise 1: Create a Simple AI Agent
1. Create an AI agent that subscribes to sensor data
2. Implement basic decision-making logic
3. Publish commands based on the agent's decisions
4. Test the agent in a simulated environment

### Exercise 2: Implement Stateful Behavior
1. Create an AI agent that maintains state between decisions
2. Implement a finite state machine for the agent
3. Add transitions between states based on sensor inputs
4. Test state transitions with various input scenarios

### Exercise 3: Error Handling
1. Add comprehensive error handling to your AI agent
2. Implement fallback behaviors for different error conditions
3. Test error scenarios to ensure robust operation
4. Log errors and publish error notifications

## References and Citations

1. [ROS 2 rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/) - Official documentation for the Python client library
2. [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Node-Composition-Python.html) - Python-specific tutorials for ROS 2
3. [ROS 2 Quality of Service in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Quality-Of-Service-Python.html) - QoS settings for Python nodes
4. [Robotics with ROS 2 and Python](https://navigation.ros.org/tutorials/docs/get_back_to_center.html) - Practical examples of Python in robotics
5. [ROS 2 Actions with Python](https://docs.ros.org/en/humble/Tutorials/Actions/Using-Actions-Python.html) - For long-running tasks with feedback

## Summary

In this chapter, you've learned how to integrate Python-based AI agents with ROS 2 systems using rclpy. You've seen examples of basic and advanced agent patterns, error handling techniques, and integration approaches. These concepts enable you to create intelligent robotic systems that can process sensor data, make decisions, and control robot behavior.