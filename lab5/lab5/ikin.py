#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from math import acos, asin, atan2, sin, cos, pi, atan, sqrt
import numpy as np

import yaml
from ament_index_python.packages import get_package_share_directory
import os

class Ikin(Node):

    def __init__(self):
        super().__init__('ikin_node')

        qos_profile = QoSProfile(depth=10)

        self.pose_stamped_sub = self.create_subscription(PoseStamped, '/position', self.position_callback, 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

        self.joint_states = JointState()

        self.params = read_params()
        self.joints = [0.0,0.0,0.0]
    
        self.clock = self.create_timer(1, self.publish_joint_states)
        self.clock

    
    def position_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        try:
            u_case = self.un_calc(x,y,z)
            idx = self.choose_un_calc(u_case)
            self.joints = u_case[idx]

            self.publish_joint_states()
        except Exception:
            self.get_logger().error("Nie można osiągnąć zadanej pozycji")

    
    def publish_joint_states(self):
        now = self.get_clock().now()
        self.joint_states.header.stamp = now.to_msg()
        self.joint_states.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']

        self.joint_states.position = [float(self.joints[0]) , float(self.joints[1]), float(self.joints[2]) ]

        self.joint_state_pub.publish(self.joint_states)

    def choose_un_calc(self, u_case):
        ths = self.joints
        uc1 = u_case[0]
        max_distance1 = max((ths[0]-uc1[0])**2,(ths[1]-uc1[1])**2,(ths[2]-uc1[2])**2)
        uc2 = u_case[1]
        max_distance2 = max((ths[0]-uc2[0])**2,(ths[1]-uc2[1])**2,(ths[2]-uc2[2])**2)
        uc3 = u_case[2]
        max_distance3 = max((ths[0]-uc3[0])**2,(ths[1]-uc3[1])**2,(ths[2]-uc3[2])**2)
        uc4 = u_case[3]
        max_distance4 = max((ths[0]-uc4[0])**2,(ths[1]-uc4[1])**2,(ths[2]-uc4[2])**2)
        value = min(max_distance1,max_distance2,max_distance3,max_distance4)

        if value == max_distance1:
            return 0
        if value == max_distance2:
            return 1
        if value == max_distance3:
            return 2
        if value == max_distance4:
            return 3


    def un_calc(self, x, y, z):
        return [self.un_calc1(x,y,z),self.un_calc2(x,y,z),self.un_calc3(x,y,z),self.un_calc4(x,y,z)]

    def un_calc1(self, x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]

        t1 = atan2(y,x)
        t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
        t2 = -asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
        return (t1,t2,t3)

    def un_calc2(self,x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]

        t1 = atan2(y,x)
        t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
        t2 = -asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
        return (t1,t2,t3)

    def un_calc3(self,x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]

        t1 = atan2(y,x)+pi
        t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
        t2 = pi - asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))

        return (t1,t2,t3)

    def un_calc4(self,x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]

        t1 = atan2(y,x)+pi
        t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
        t2 = pi - asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))

        return (t1,t2,t3)

def read_params():
    with open(os.path.join(get_package_share_directory('lab5'), 'params.yaml'), 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)

        return params

def main():
    rclpy.init()
    ikin = Ikin()
    try:
        rclpy.spin(ikin)
    except KeyboardInterrupt:
        ikin.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
  main()