import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Jint
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import math
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

        try:
            self.request_check(request)

            if request.interpolation_type == 'Linear':
                self.linear_interpolation(request.joint1, request.joint2, request.joint3, request.time)
            elif request.interpolation_type == 'Spline':
                self.spline_interpolation(request.joint1, request.joint2, request.joint3, request.time)

            response.response = "Interpolacja zakończona pomyślnie"
            return response
        except ValueError as e:
            response.response = "Interpolacja niemożliwa. " + e.args[0]
            return response
        
    def linear_interpolation(self, req_pos1, req_pos2, req_pos3, int_time):
        joint_state = JointState()
        moves = (int)(int_time/self.rate)
        increment1 = (req_pos1 - self.start_position1)/moves
        increment2 = (req_pos2 - self.start_position2)/moves
        increment3 = (req_pos3 - self.start_position3)/moves

        for i in range(moves):
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']
            
            self.position1 += increment1
            self.position2 += increment2
            self.position3 += increment3

            joint_state.position = [float(self.position1),float(self.position2),float(self.position3)]
            self.publisher.publish(joint_state)
            time.sleep(self.rate)
        


    def spline_interpolation(self, req_pos1, req_pos2, req_pos3, int_time):
        joint_state = JointState()
        moves = (int)(int_time/self.rate)
        a0_1 = self.position1
        a0_2 = self.position2
        a0_3 = self.position3

        a1_1 = 0
        a1_2 = 0
        a1_3 = 0

        a2_1 = 3*(req_pos1-self.position1)/(int_time**2)
        a2_2 = 3*(req_pos2-self.position2)/(int_time**2)
        a2_3 = 3*(req_pos3-self.position3)/(int_time**2)

        a3_1 = -2*(req_pos1-self.position1)/(int_time**3)
        a3_2 = -2*(req_pos2-self.position2)/(int_time**3)
        a3_3 = -2*(req_pos3-self.position3)/(int_time**3)

        for i in range(moves):
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']
            
            self.position1 = a0_1 + a1_1*(i*self.rate) + a2_1*(i*self.rate)**2 + a3_1*(i*self.rate)**3
            self.position2 = a0_2 + a1_2*(i*self.rate) + a2_2*(i*self.rate)**2 + a3_2*(i*self.rate)**3
            self.position3 = a0_3 + a1_3*(i*self.rate) + a2_3*(i*self.rate)**2 + a3_3*(i*self.rate)**3

            joint_state.position = [float(self.position1),float(self.position2),float(self.position3)]
            self.publisher.publish(joint_state)
            time.sleep(self.rate)

    def request_check(self, request):
        if(request.joint1 < 0 or request.joint1 > 2*math.pi):
            err = 'Niepoprawna wartośc dla stawu 1'
            self.get_logger().error(err) 
            raise ValueError(err)

        if(request.joint2 > 0 or request.joint2 < -math.pi):
            err = 'Niepoprawna wartośc dla stawu 2'
            self.get_logger().error(err)               
            raise ValueError(err)

        if(request.joint3 > 3 or request.joint3 < -3):
            err = 'Niepoprawna wartośc dla stawu 3'
            self.get_logger().error(err)
            raise ValueError(err)

        if(request.time <= 0):
            err = 'Niepoprawna wartość czasu'
            self.get_logger().error(err)
            raise ValueError(err)

        if(request.interpolation_type != 'Linear' and request.interpolation_type != 'Spline'):
            err = 'Zły typ interpolacji'
            self.get_logger().error(err)
            raise ValueError(err)
        

def main():
    rclpy.init()
    minimal_service = MinimalService()
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()