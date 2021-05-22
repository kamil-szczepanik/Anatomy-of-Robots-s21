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
        self.positions = [2.0,0,0.05]
    
        self.clock = self.create_timer(1, self.publish_joint_states)
        self.clock

    
    def position_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        u_case = self.un_calc(x,y,z)

        # TUTAJ WYBIERA PIERWSZĄ OPCJE:
        # TODO: wybranie opcji, która najmniej zmienia położenie jointów
        self.positions = u_case[0]

        self.publish_joint_states()

    
    def publish_joint_states(self):
        now = self.get_clock().now()
        self.joint_states.header.stamp = now.to_msg()
        self.joint_states.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']

        self.joint_states.position = [float(self.positions[0]) , float(self.positions[1]), float(self.positions[2]) ]

        self.joint_state_pub.publish(self.joint_states)


    def un_calc(self, x, y, z):
        return [self.un_calc1(x,y,z),self.un_calc2(x,y,z),self.un_calc3(x,y,z),self.un_calc4(x,y,z)]

    def un_calc1(self, x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]
        try:
            t1 = atan2(y,x)
            t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
            t2 = -asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
        except Exception:
            print("1\n")
            t1=t2=t3=0
        return (t1,t2,t3)

    def un_calc2(self,x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]
        try:
            t1 = atan2(y,x)
            t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
            t2 = -asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
        except Exception:
            print("2\n")
            t1=t2=t3=0
        return (t1,t2,t3)

    def un_calc3(self,x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]
        try:
            t1 = atan2(y,x)+pi
            t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
            t2 = pi - asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))
        except Exception:
            print("3\n")
            t1=t2=t3=0
        return (t1,t2,t3)

    def un_calc4(self,x,y,z):
        params = self.params
        a2 = params["arm"]["a"]
        a3 = params["hand"]["a"]
        d1 = params["base"]["d"]
        try:
            t1 = atan2(y,x)+pi
            t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
            t2 = pi - asin(a3*sin(t3)/sqrt((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))
        except Exception:
            print("4\n")
            t1=t2=t3=0
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