import sys

import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Jint

from geometry_msgs.msg import JointState


# Service - dwa parametry: czas w jakim robot ma się przemieścić i punkt w ktorym ma byc końcowka

class JintClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.srv = self.create_client(Jint, 'jint_control_srv')
        self.joint_state_pub = self.create_publisher(Joint, '/joint_states', qos_profile)
        

                

def main(args=None):
    rclpy.init(args=args)

    client = JintClient()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    "Service call failed %r" % (e,))
            else:
                client.get_logger().info(
                    "Result of interpolation: %d" % (response.response))
            break

    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()