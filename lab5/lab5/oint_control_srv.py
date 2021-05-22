import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

from visualization_msgs.msg import Marker, MarkerArray

from math import sin, cos, sqrt
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import os
from interpolation_srv.srv import OintXYZ

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(OintXYZ, 'oint_control_srv', self.oint_control_srv_callback)
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(PoseStamped, "/position", qos_profile)
        self.marker_pub = self.create_publisher(Marker, "/path_marker", qos_profile)
        self.robot_params = read_params()

        self.rate = 0.1
        # initial position
        self.pos_x = 2.0
        self.pos_y = 0.0
        self.pos_z = 0.05

        self.clock = self.create_timer(1, self.publish_position)
        self.clock


    def oint_control_srv_callback(self, request, response):

        self.get_logger().info('Incoming request\n'+
                                f' - x: {request.x}\n' +
                                f' - y: {request.y}\n'+
                                f' - z: {request.z}\n'+
                                f' -- time: {request.time}\n'+
                                f' --- interpolation type: {request.interpolation_type}')

        try:
            self.request_check(request)
            self.pose_stamped = PoseStamped()
            self.pose_stamped.header.frame_id = "base"
            self.marker_init()

            if request.interpolation_type == "Linear":
                self.linear_interpolation(request.x, request.y, request.z, request.time)
            elif request.interpolation_type == "Polynomial":
                self.polynomial_interpolation(request.x, request.y, request.z, request.time)
            
            response.response = "Interpolacja zakonczona pomyslnie"
            return response
        
        except ValueError as e:
            response.response = "Interpolacja niemozliwa. " + e.args[0]
            return response
        
    def linear_interpolation(self, req_x, req_y, req_z, int_time):

        moves = (int)(int_time/self.rate)    

        pos_x_increment = (req_x - self.pos_x)/moves
        pos_y_increment = (req_y - self.pos_y)/moves
        pos_z_increment = (req_z - self.pos_z)/moves

        for i in range(moves):
            now = self.get_clock().now()
            self.pose_stamped.header.stamp = now.to_msg()
            
            self.pos_x += pos_x_increment
            self.pos_y += pos_y_increment
            self.pos_z += pos_z_increment

            self.pose_stamped.pose.position = Point(x=float(self.pos_x), y=float(self.pos_y), z=float(self.pos_z))
            
            self.publisher.publish(self.pose_stamped)

            self.marker_show()

            time.sleep(self.rate)

    def polynomial_interpolation(self, req_x, req_y, req_z, int_time):
        
        moves = (int)(int_time/self.rate)

        a0_x = self.pos_x
        a0_y = self.pos_y
        a0_z = self.pos_z
        a1_x = 0
        a1_y = 0
        a1_z = 0
        a2_x = 3*(req_x - self.pos_x)/(int_time**2)
        a2_y = 3*(req_y - self.pos_y)/(int_time**2)
        a2_z = 3*(req_z - self.pos_z)/(int_time**2)
        a3_x = -2*(req_x - self.pos_x)/(int_time**3)
        a3_y = -2*(req_y - self.pos_y)/(int_time**3)
        a3_z = -2*(req_z - self.pos_z)/(int_time**3)

        for i in range(moves):
            now = self.get_clock().now()
            self.pose_stamped.header.stamp = now.to_msg()
            
            self.pos_x = a0_x + a1_x*(i*self.rate) + a2_x*(i*self.rate)**2 + a3_x*(i*self.rate)**3
            self.pos_y = a0_y + a1_y*(i*self.rate) + a2_y*(i*self.rate)**2 + a3_y*(i*self.rate)**3
            self.pos_z = a0_z + a1_z*(i*self.rate) + a2_z*(i*self.rate)**2 + a3_z*(i*self.rate)**3

            self.pose_stamped.pose.position = Point(x=float(self.pos_x), y=float(self.pos_y), z=float(self.pos_z))

            self.publisher.publish(self.pose_stamped)

            self.marker_show()

            time.sleep(self.rate)


    def request_check(self, request):
        if(request.z < 0):
            err = 'Niepoprawna wartość z'
            self.get_logger().error(err) 
            raise ValueError(err)

        req_distance = sqrt(request.x**2+request.y**2+(request.z-self.robot_params['base']['d'])**2)
        possible_distance = self.robot_params['arm']['a']+self.robot_params['hand']['a']
        if(req_distance > possible_distance):
            err = 'Cel nieosiągalny'
            self.get_logger().error(err)               
            raise ValueError(err)

        if(request.time <= 0):
            err = 'Niepoprawna wartość czasu'
            self.get_logger().error(err)
            raise ValueError(err)

        if(request.interpolation_type != 'Linear' and request.interpolation_type != 'Polynomial'):
            err = 'Zly typ interpolacji'
            self.get_logger().error(err)
            raise ValueError(err)

    def marker_init(self):

        self.marker = Marker()
        self.marker.id  = 0
        self.marker.action = Marker.DELETEALL
        self.marker.header.frame_id = "base"
        self.marker.header.stamp

        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05
        self.marker.color.a = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0

        self.marker_pub.publish(self.marker)

    def make_point(self):
        p = Point()
        p.x = self.pos_x
        p.y = self.pos_y
        p.z = self.pos_z
        return p

    def marker_show(self):
        self.path_point = self.make_point()
        self.marker.points.append(self.path_point)
        self.marker_pub.publish(self.marker)

    def publish_position(self):
        pose_stamped = PoseStamped()
        now = self.get_clock().now()
        pose_stamped.header.stamp = now.to_msg()
        pose_stamped.header.frame_id = "base"
        pose_stamped.pose.position = Point(x=self.pos_x, y=self.pos_y, z=self.pos_z)
        self.publisher.publish(pose_stamped)


def read_params():
    with open(os.path.join(get_package_share_directory('lab5'), 'params.yaml'), 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)

        return params


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()