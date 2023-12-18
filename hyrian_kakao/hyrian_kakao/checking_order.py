#!/usr/bin/env python3
import subprocess
import time
import json
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)

    def publish_message(self, msg):
        msg = String()
        msg.data = 'Status changed to AUTH_PASSWORD'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()

    while True:
      output = subprocess.run(['./test2.sh'], capture_output=True, text=True).stdout
      print('Output:', output)

      # Extract JSON part from the output
      match = re.search(r'\{.*\}', output)
      if match:
        json_part = match.group()
        status = json.loads(json_part)['status']

        if status == 'AUTH_PASSWORD':
            my_publisher.publish_message('Status changed to AUTH_PASSWORD')
            break

    time.sleep(10000)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()