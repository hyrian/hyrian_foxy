import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeywordSubscriber(Node):
    def __init__(self):
        super().__init__('keyword_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/keyword_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    keyword_subscriber = KeywordSubscriber()
    rclpy.spin(keyword_subscriber)
    keyword_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()