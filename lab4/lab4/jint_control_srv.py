import rclpy
from rclpy.node import Node
from interpolation_srv.srv import Jint
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from math import sin, cos, pi
import numpy as np
import mathutils

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Jint, 'jint_control_srv', self.jint_control_srv_callback)
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, "/joint_states", qos_profile)
        self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile)
        self.robot_params = read_params()

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
   
    def jint_control_srv_callback(self, request, response):
        self.marker_init()

        self.start_position1 = self.position1
        self.start_position2 = self.position2
        self.start_position3 = self.position3

        self.get_logger().info('Nadchodzące żądanie\n'+
                                f' - pozycja 1: {request.joint1}\n' +
                                f' - pozycja 2: {request.joint2}\n'+
                                f' - pozycja 3: {request.joint3}\n'+
                                f' -- czas: {request.time}\n'+
                                f' --- typ interpolacji: {request.interpolation_type}')

        try:
            self.request_check(request)

            if request.interpolation_type == 'Linear':
                self.linear_interpolation(request.joint1, request.joint2, request.joint3, request.time)
            elif request.interpolation_type == 'Polynomial':
                self.polynomial_interpolation(request.joint1, request.joint2, request.joint3, request.time)

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

            self.marker_show()

            time.sleep(self.rate)
        
    def polynomial_interpolation(self, req_pos1, req_pos2, req_pos3, int_time):
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

            self.marker_show()

            time.sleep(self.rate)

    def request_check(self, request):
        if(request.joint1 < 0 or request.joint1 > 2*pi):
            err = 'Niepoprawna wartośc dla stawu 1'
            self.get_logger().error(err) 
            raise ValueError(err)

        if(request.joint2 > 0 or request.joint2 < -pi):
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

        if(request.interpolation_type != 'Linear' and request.interpolation_type != 'Polynomial'):
            err = 'Zły typ interpolacji'
            self.get_logger().error(err)
            raise ValueError(err)
        
    def marker_init(self):
        self.markerArray = MarkerArray()

        self.marker = Marker()
        self.marker.header.frame_id = "base"

        self.marker.id = 0
        self.marker.action = Marker.DELETEALL
        self.markerArray.markers.append(self.marker)
        self.marker_pub.publish(self.markerArray)
        
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05
        self.marker.color.a = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.orientation.x = 1.0
        self.marker.pose.orientation.y = 1.0
        self.marker.pose.orientation.z = 1.0
    
    def marker_show(self):
        xyz = self.calc_position()
        self.marker.pose.position.x = xyz[0]
        self.marker.pose.position.y = xyz[1]
        self.marker.pose.position.z = xyz[2]
        self.markerArray.markers.append(self.marker)

        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        self.marker_pub.publish(self.markerArray)

    def calc_position(self):

        params = self.robot_params
        positions = [self.position1, self.position2, self.position3, 0]
        T = np.eye(4)

        for i in range(len(params.keys())):

            theta = positions[i]

            if i == 0:
                link = 'base'
            if i == 1:
                link = 'base_ext'
            if i == 2:
                link = 'arm'
            if i == 3:
                link = 'hand'
            
            d = params[link]['d']
            a = params[link]['a']
            alpha = params[link]['alpha']
            
            
            Rotx = np.array([[1, 0, 0, 0],
                                [0, cos(alpha), -sin(alpha), 0],
                                [0, sin(alpha), cos(alpha), 0],
                                [0, 0, 0, 1]])

            Transx = np.array([[1, 0, 0, a],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

            Rotz = np.array([[cos(theta), -sin(theta), 0, 0],
                                [sin(theta), cos(theta), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

            Transz = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d],
                                [0, 0, 0, 1]])

            T_curr = Rotx@Transx@Rotz@Transz
            T = T @ T_curr

        xyz = [T[0][3], T[1][3], T[2][3]]

        return xyz


def read_params():
    with open(os.path.join(get_package_share_directory('lab4'), 'params.yaml'), 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)

    return params


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