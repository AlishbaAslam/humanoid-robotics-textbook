# Chapter 1: Fundamentals of ROS 2 Nodes, Topics, and Services

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is not an operating system, but rather a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 is designed to support the development of complex robotic systems by providing:
- Distributed computing capabilities
- Package management for code sharing
- Hardware abstraction
- Device drivers
- Libraries for common robot functions
- Message-passing between processes
- Tools for building and testing code

## Core Concepts

### Nodes

A **Node** is the fundamental unit of computation in ROS 2. It's a process that performs computation. Nodes are organized into packages to make the sharing of code easier.

Key characteristics of nodes:
- Each node runs a specific task
- Nodes communicate with other nodes through messages
- Nodes can be written in different programming languages (Python, C++, etc.)
- A single program can contain multiple nodes

Example of a simple node structure:
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Message Passing

**Topics** are named buses over which nodes exchange messages. ROS 2 uses a publish/subscribe messaging model where nodes can publish messages to a topic or subscribe to a topic to receive messages.

Key concepts:
- Topics enable asynchronous communication
- Multiple publishers and subscribers can exist for the same topic
- Messages are typed according to a message definition
- Data flows from publishers to subscribers

### Services

**Services** provide a request/response communication pattern between nodes. Unlike topics which are asynchronous, services are synchronous and block until a response is received.

Service structure:
- Request message from client to server
- Response message from server to client
- Defined by a service definition file (.srv)
- Useful for tasks that require immediate responses

## Practical Examples

Let's explore these concepts with practical examples:

### Basic Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BasicPublisher(Node):
    def __init__(self):
        super().__init__('basic_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    basic_publisher = BasicPublisher()
    rclpy.spin(basic_publisher)
    basic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BasicSubscriber(Node):
    def __init__(self):
        super().__init__('basic_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    basic_subscriber = BasicSubscriber()
    rclpy.spin(basic_subscriber)
    basic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Service Server

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Patterns and Best Practices

### Publisher-Subscriber Pattern
- Use appropriate QoS (Quality of Service) settings for your application
- Consider message frequency and bandwidth requirements
- Use latching for static messages that late-joining subscribers need

### Client-Server Pattern
- Use services for synchronous operations
- Consider using actions for long-running tasks with feedback
- Handle service call timeouts appropriately

## Exercises

### Exercise 1: Create a Publisher-Subscriber Pair
1. Create a publisher node that publishes temperature readings
2. Create a subscriber node that logs these readings
3. Run both nodes and verify they communicate correctly

### Exercise 2: Create a Simple Service
1. Create a service server that calculates the area of a rectangle
2. Create a client that calls this service with different dimensions
3. Verify that the service returns correct results

## References and Citations

1. [ROS 2 Documentation - Concepts](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Concepts.html) - Official ROS 2 documentation on core concepts
2. [ROS 2 Tutorials - Writing a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) - Official tutorial for basic publisher/subscriber implementation
3. [ROS 2 Documentation - Services](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Interfaces.html#services) - Detailed explanation of services in ROS 2
4. [ROS 2 Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) - Information about QoS settings for different applications
5. [ROS 2 Design](https://design.ros2.org/) - Design documents explaining the architecture of ROS 2

## Summary

In this chapter, you've learned about the fundamental concepts of ROS 2 including nodes, topics, and services. You've seen practical examples of each concept and learned about common patterns and best practices. These concepts form the foundation for more advanced ROS 2 development, which we'll explore in the following chapters.