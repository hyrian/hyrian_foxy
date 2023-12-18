import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
from hyrian_interfaces.srv import Kakaopaysend
import subprocess

class KakaopayNode(Node):
    def __init__(self):
        super().__init__('kakaopay_node')
        self.service_ = self.create_service(Kakaopaysend, 'kakaopay_send_service', self.handle_service)
        self.subscription_ = self.create_subscription(String, 'my_topic', self.topic_callback, 10)
        self.prod = ""
        self.response_ = None

    def handle_service(self, request, response):
        self.prod = request.gesture
        if self.prod == "product1":
            subprocess.run(["bash", "get_output.sh"])
            self.response_ = response
        return response

    def topic_callback(self, msg):
        if msg.data == "Status changed to AUTH_PASSWORD":
            self.response_.finish = "done"
            self.get_logger().info('Service response: "%s"' % self.response_.finish)

def main(args=None):
    rclpy.init(args=args)
    kakaopay_node = KakaopayNode()
    rclpy.spin(kakaopay_node)
    kakaopay_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()