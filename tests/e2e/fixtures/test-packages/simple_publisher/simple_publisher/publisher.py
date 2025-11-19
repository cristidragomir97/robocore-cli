#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import socket
import os
from datetime import datetime


class TestPublisher(Node):
    """Test publisher that publishes system info and test data."""

    def __init__(self):
        super().__init__('simple_test_publisher')

        # Get parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('topic_name', '/test_topic')

        publish_rate = self.get_parameter('publish_rate').value
        topic_name = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.counter = 0
        self.hostname = socket.gethostname()
        self.container_id = os.environ.get('HOSTNAME', 'unknown')

        self.get_logger().info(f'Test publisher started on {self.hostname}')
        self.get_logger().info(f'Publishing to {topic_name} at {publish_rate} Hz')

    def timer_callback(self):
        """Publish test message with system info."""
        msg = String()

        # Create test data with system information
        test_data = {
            'counter': self.counter,
            'timestamp': datetime.now().isoformat(),
            'hostname': self.hostname,
            'container_id': self.container_id,
            'node_name': self.get_name(),
            'pid': os.getpid(),
            'message': f'Hello from robocore-cli test #{self.counter}'
        }

        msg.data = json.dumps(test_data, indent=2)
        self.publisher_.publish(msg)

        if self.counter % 10 == 0:  # Log every 10th message
            self.get_logger().info(f'Published message #{self.counter} from {self.hostname}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TestPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()