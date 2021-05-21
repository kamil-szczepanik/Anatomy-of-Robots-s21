#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from math import acos, asin, atan2, sin, cos, pi, atan, sqrt
import numpy as np


class Ikin(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('ikin_node')

        qos_profile = QoSProfile(depth=10)

        self.pose_stamped_sub = self.create_subscription(PoseStamped, '/position', self.position_callback, 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

        self.joint_states = JointState()

    
    def position_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        u_case = un_calc(x,y,z)

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



# TODO: parametry wczytywane z serwera 
params = {"base":{"a":0,"d":0.05,"alpha":0}, 
	"base_ext":{"a":0,"d":0,"alpha":-pi/2},
	"arm":{"a":1,"d":0,"alpha":0},
	"hand":{"a":0.5,"d":0,"alpha":0}}

def calc(t1, t2 ,t3):

	positions = [t1, t2, t3, 0]
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

	xyz = [round(T[0][3],4), round(T[1][3],4), round(T[2][3],4)]

	return xyz

def un_calc(x, y, z):
	return [un_calc1(x,y,z),un_calc2(x,y,z),un_calc3(x,y,z),un_calc4(x,y,z)]

def un_calc1(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)
		t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = -asin(a3*sin(t3)/((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("1\n")
		t1=t2=t3=0
	return (t1,t2,t3)

def un_calc2(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)
		t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = -asin(a3*sin(t3)/((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("2\n")
		t1=t2=t3=0
	return (t1,t2,t3)

def un_calc3(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)+pi
		t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = pi - asin(a3*sin(t3)/((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("3\n")
		t1=t2=t3=0
	return (t1,t2,t3)

def un_calc4(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)+pi
		t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = pi - asin(a3*sin(t3)/((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("4\n")
		t1=t2=t3=0
	return (t1,t2,t3)

# u_case = un_calc(0,1,0)

# print(u_case[0],calc(*u_case[0]))
# print(u_case[1],calc(*u_case[1]))
# print(u_case[2],calc(*u_case[2]))
# print(u_case[3],calc(*u_case[3]))

def main(args=None):
  try:
    node = Ikin()
    rclpy.spin(node)
  except Exception as e:
    print(e)
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()