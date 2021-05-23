import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

from visualization_msgs.msg import Marker, MarkerArray

from math import sin, cos, sqrt, pi
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import os
from interpolation_srv.srv import OintXYZ
from interpolation_srv.srv import TrajectoryPath

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(OintXYZ, 'oint_control_srv', self.oint_control_srv_callback)
        self.trajectory_srv = self.create_service(TrajectoryPath, 'trajectory_srv', self.trajectory_srv_callback)
        
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

        self.get_logger().info('Nadchodzące żądanie\n'+
                                f' - x: {request.x}\n' +
                                f' - y: {request.y}\n'+
                                f' - z: {request.z}\n'+
                                f' -- czas: {request.time}\n'+
                                f' --- typ interpolacji: {request.interpolation_type}')

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

    def trajectory_srv_callback(self, request, response):
        self.get_logger().info('Nadchodzące żądanie\n'+
                        f' - typ trajektorii: {request.trajectory_type}\n' +
                        f' - parametr a: {request.param_a}\n'+
                        f' - parametr b: {request.param_b}\n'+
                        f' -- czas: {request.time}\n'+
                        f' --- typ interpolacji: {request.interpolation_type}')

        try:
            self.request_check_interpolation_type(request)
            self.request_check_trajectory_type(request)
            self.request_check_time(request)
            self.pose_stamped = PoseStamped()
            self.pose_stamped.header.frame_id = "base"
            self.marker_init()

            if request.trajectory_type == "Rectangle":
                if not self.request_check_rectangle_points(request):
                    raise ValueError("Punkty prostokąta poza zasiegiem")
                else:
                    self.draw_rectangle(request)

            elif request.trajectory_type == "Ellipse":
                if not self.request_check_ellipse(request):
                    raise ValueError("Punkty elipsy poza zasiegiem")
                else:
                    self.draw_ellipse(request)

            response.response = "Interpolacja zakonczona pomyslnie"
            return response
        
        except ValueError as e:
            response.response = "Interpolacja niemozliwa. " + e.args[0]
            return response


    
    def request_check_interpolation_type(self, request):
        if(request.interpolation_type != 'Linear' and request.interpolation_type != 'Polynomial'):
            err = 'Zly typ interpolacji'
            self.get_logger().error(err)
            raise ValueError(err)

    def request_check_trajectory_type(self, request):
        if(request.trajectory_type != 'Rectangle' and request.trajectory_type != 'Ellipse'):
            err = 'Zly typ trajektorii'
            self.get_logger().error(err)
            raise ValueError(err)

    def request_check_rectangle_points(self, request):
        inRange = True
        points = self.find_rectangle_points(request)
        for point in points:
            if not self.check_if_goal_is_in_range([point.x, point.y, point.z])>0:
                inRange = False
        return inRange


    def find_rectangle_points(self, request):
        A = Point(x=self.pos_x , y=self.pos_y, z=self.pos_z)
        B = Point(x=self.pos_x + request.param_a, y=self.pos_y, z=self.pos_z)
        C = Point(x=self.pos_x + request.param_a, y=self.pos_y, z=self.pos_z - request.param_b)
        D = Point(x=self.pos_x, y=self.pos_y, z=self.pos_z - request.param_b)
        return [A, B, C, D]

    def find_ellipse_points(self, request):
        moves = (int)(request.time/self.rate)
        points = []
        for i in range(moves):
            centre_x = self.pos_x-request.param_a
            new_point = Point(x=centre_x+request.param_a*cos(2*pi*i/moves), y=self.pos_y, z=self.pos_z + request.param_b*sin(2*pi*i/moves))
            points.append(new_point)
        return points
    

    def request_check_ellipse(self, request):
        inRange = True
        points = self.find_ellipse_points(request)
        for point in points:
            if not self.check_if_goal_is_in_range([point.x, point.y, point.z]):
                inRange = False
        return inRange


    def request_check_time(self, request):
        if(request.time <= 0):
            err = 'Niepoprawna wartość czasu'
            self.get_logger().error(err)
            raise ValueError(err)


    def check_if_goal_is_in_range(self, goal):
 
        x = goal[0]
        y = goal[1]
        z = goal[2]
 
        sphere_centre = [0, 0, self.robot_params["base"]["d"]]
        c_x = sphere_centre[0]
        c_y = sphere_centre[1]
        c_z = sphere_centre[2]
        radius = self.robot_params["arm"]["a"] + self.robot_params["hand"]["a"]
 
        return ( (x-c_x)**2 + (y-c_y)**2 + (z-c_z)**2 <= radius**2 and z>0)
    
    def draw_rectangle(self, request):
        self.marker_show()
        quarter = request.time/4
        points = self.find_rectangle_points(request)
        if request.interpolation_type == "Linear":
            self.linear_interpolation(points[1].x, points[1].y, points[1].z, quarter)
            self.linear_interpolation(points[2].x, points[2].y, points[2].z, quarter)
            self.linear_interpolation(points[3].x, points[3].y, points[3].z, quarter)
            self.linear_interpolation(points[0].x, points[0].y, points[0].z, quarter)
        elif request.interpolation_type == "Polynomial":
            self.polynomial_interpolation(points[1].x, points[1].y, points[1].z, quarter)
            self.polynomial_interpolation(points[2].x, points[2].y, points[2].z, quarter)
            self.polynomial_interpolation(points[3].x, points[3].y, points[3].z, quarter)
            self.polynomial_interpolation(points[0].x, points[0].y, points[0].z, quarter)
    
    def draw_ellipse(self, request):
        self.marker_show()
        moves = (int)(request.time/self.rate)
        time_fragment = request.time/moves
        points = self.find_ellipse_points(request)
        if request.interpolation_type == "Linear":
            for point in points:
                self.linear_interpolation(point.x , point.y, point.z, time_fragment)



 
 


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