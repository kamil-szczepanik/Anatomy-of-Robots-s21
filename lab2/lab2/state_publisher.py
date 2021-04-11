#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class StatePublisher(Node):

  def __init__(self):
    rclpy.init()
    super().__init__('state_publisher')

    qos_profile = QoSProfile(depth=10)
    self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
    self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
    self.delay = self.create_subscription(JointState,'joint_states', self.stop, 10)
    self.delay
    self.nodeName = self.get_name()
    self.sent = False

    self.get_logger().info("{0} started".format(self.nodeName))
    self.params = {}
    self.declare_parameters(
      namespace='',
      parameters=[
        ('theta1', 0.0),
        ('theta2', -1.57),
        ('theta3', 1.57),
        ('theta4', 0.0),
        ('a1', 0.0),
        ('d1', 0.0),
        ('a2', 1.0),
        ('d2', 0.0),
        ('a3', 0.5),
        ('d3', 0.0)
      ])
  
    self.params['theta1'] = self.get_parameter('theta1')._value
    self.params['theta2'] = self.get_parameter('theta2')._value
    self.params['theta3'] = self.get_parameter('theta3')._value
    self.params['theta4'] = self.get_parameter('theta4')._value
    self.params['a1'] = self.get_parameter('a1')._value
    self.params['d1'] = self.get_parameter('d1')._value
    self.params['a2'] = self.get_parameter('a2')._value
    self.params['d2'] = self.get_parameter('d2')._value
    self.params['a3'] = self.get_parameter('a3')._value
    self.params['d3'] = self.get_parameter('d3')._value
    

    # robot state
    self.theta1 = self.params['theta1']
    self.theta2 = self.params['theta2']
    self.theta3 = self.params['theta3']
    self.theta4 = self.params['theta4']

    # message declarations
    self.odom_trans = TransformStamped()
    self.odom_trans.header.frame_id = 'base'
    self.odom_trans.child_frame_id = 'base_ext'
    self.joint_state = JointState()
  

  
    now = self.get_clock().now()
    self.joint_state.header.stamp = now.to_msg()
    self.joint_state.name = ['base-base_ext', 'base_ext-arm', 'arm-hand', 'hand-tool']
    self.joint_state.position = [self.theta1, self.theta2, self.theta3, self.theta4]

    # update transform
    
    self.odom_trans.header.stamp = now.to_msg()
    self.odom_trans.transform.translation.x = 0.0
    self.odom_trans.transform.translation.y = 0.0
    self.odom_trans.transform.translation.z = 0.0
    self.odom_trans.transform.rotation = \
        euler_to_quaternion(0, 0, pi/2) # roll,pitch,yaw
        
    # send the transform
    self.broadcaster.sendTransform(self.odom_trans)
    
    while not self.sent:
      self.move()

  def move(self):

    # send the joint state
    self.joint_pub.publish(self.joint_state)
    

  def stop(self, msg):
    self.sent = True

    

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  try:
    node = StatePublisher()
    
    rclpy.spin_once(node)
    

    node.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    pass

if __name__ == '__main__':
  main()