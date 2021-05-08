import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Jint

from geometry_msgs.msg import JointState
from rclpy.qos import QoSProfile

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
                                ' - position1: %d\n' +
                                ' - position2: %d\n'+
                                ' - position3: %d\n'+
                                ' -- time: %d\n'+
                                ' --- interpolation type:%d' % (request.position1, 
                                                                request.position2, 
                                                                request.position3, 
                                                                request.time, 
                                                                request.interpolation))

        joint_state = JointState()
        moves = (int)(time/self.rate)

        for i in range(moves):
            now = self.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()
            self.joint_state.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']

            if request.interpolation == 'Linear':
                self.linear_interpolation(request.position1, request.position2, request.position3, moves)
            elif request.interpolation == "Spline":
                self.spline_interpolation()

            joint_state.position = [float(self.position1),float(self.position3),float(self.position3)]
            self.publisher.publish(joint_state)
        
    def linear_interpolation(self, req_pos1, req_pos2, req_pos3, moves):
        self.position1 += (req_pos1 - self.position1)/moves
        self.position2 += (req_pos2 - self.position2)/moves
        self.position3 += (req_pos3 - self.position3)/moves


    def spline_interpolation(self):
        pass



def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()