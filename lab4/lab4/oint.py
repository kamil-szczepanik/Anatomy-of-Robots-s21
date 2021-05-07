import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Oint

from geometry_msgs.msg import JointState


# Service - dwa parametry: czas w jakim robot ma się przemieścić i punkt w ktorym ma byc końcowka

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.srv = self.create_client(Oint, 'oint_control_srv')
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

                

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()