import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Jint
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import time

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Jint, 'jint_control_srv', self.jint_control_srv_callback)
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, "/joint_states", qos_profile)

        self.declare_parameters(
                namespace='',
                parameters=[
                    ('position1', 0.0),
                    ('position2', 0.0),
                    ('position3', 0.0),
                ])
        self.position1 = self.get_parameter('position1').get_parameter_value().double_value
        self.position2 = self.get_parameter('position2').get_parameter_value().double_value
        self.position3 = self.get_parameter('position3').get_parameter_value().double_value
    
        self.rate = 0.1
        self.start_position1 = self.position1
        self.start_position2 = self.position2
        self.start_position3 = self.position3
   
    def jint_control_srv_callback(self, request, response):

        self.get_logger().info('Incoming request\n'+
                                f' - position1: {request.joint1}\n' +
                                f' - position2: {request.joint2}\n'+
                                f' - position3: {request.joint3}\n'+
                                f' -- time: {request.time}\n'+
                                f' --- interpolation type: {request.interpolation_type}')

        joint_state = JointState()
        moves = (int)(request.time/self.rate)
        self.start_position1 = self.position1
        self.start_position2 = self.position2
        self.start_position3 = self.position3

        for i in range(moves):
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']

            if request.interpolation_type == 'Linear':
                self.linear_interpolation(request.joint1, request.joint2, request.joint3, moves)
            elif request.interpolation_type == "Spline":
                self.spline_interpolation()

            joint_state.position = [float(self.position1),float(self.position2),float(self.position3)]
            self.publisher.publish(joint_state)
            time.sleep(self.rate)

        response.response = "Interpolacja zakończona"
        return response
        
    def linear_interpolation(self, req_pos1, req_pos2, req_pos3, moves):
        self.position1 += (req_pos1 - self.start_position1)/moves
        self.position2 += (req_pos2 - self.start_position2)/moves
        self.position3 += (req_pos3 - self.start_position3)/moves


    def spline_interpolation(self):
        pass



def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()