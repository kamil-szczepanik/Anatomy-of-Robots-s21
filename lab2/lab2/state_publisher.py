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
    self.nodeName = self.get_name()

    self.get_logger().info("{0} started".format(self.nodeName))
    self.params = {}
    self.declare_parameters(
      namespace='',
      parameters=[
        ('theta1', 0.0),
        ('theta2', 0.0),
        ('theta3', 0.0)
      ])
  
    self.params['theta1'] = self.get_parameter('theta1')._value
    self.params['theta2'] = self.get_parameter('theta2')._value
    self.params['theta3'] = self.get_parameter('theta3')._value

    #timer
    self.timer = self.create_timer(0.05, self.move)

    # robot state
    self.theta1 = self.params['theta1']
    self.theta2 = self.params['theta2']
    self.theta3 = self.params['theta3']

    # message declarations
    self.odom_trans = TransformStamped()
    self.odom_trans.header.frame_id = 'odom'
    self.odom_trans.child_frame_id = 'base'
    self.joint_state = JointState()
  

  def update_params(self):
    change = ''
    if self.params['theta1'] != self.get_parameter('theta1')._value:
      self.params['theta1'] = self.get_parameter('theta1')._value
      change = f'Zmieniono theta1 na {self.params["theta1"]}'
    if self.params['theta2'] != self.get_parameter('theta2')._value:
      self.params['theta2'] = self.get_parameter('theta2')._value
      change = f'Zmieniono theta2 na {self.params["theta2"]}'
    if self.params['theta3'] != self.get_parameter('theta3')._value:
      self.params['theta3'] = self.get_parameter('theta3')._value
      change = f'Zmieniono theta3 na {self.params["theta3"]}'
    if change != '':
      self.get_logger().info(change)
    return change


  def move(self):
    try:
      now = self.get_clock().now()
      self.joint_state.header.stamp = now.to_msg()
      self.joint_state.name = ['base-base_ext', 'base_ext-arm', 'arm-hand']
      self.joint_state.position = [self.theta1, self.theta2, self.theta3]

      # update transform
      
      self.odom_trans.header.stamp = now.to_msg()
      self.odom_trans.transform.translation.x = 0.0
      self.odom_trans.transform.translation.y = 0.0
      self.odom_trans.transform.translation.z = 0.0
      self.odom_trans.transform.rotation = \
          euler_to_quaternion(0, 0, pi/2) # roll,pitch,yaw

      # send the joint state and transform
      self.joint_pub.publish(self.joint_state)
      self.broadcaster.sendTransform(self.odom_trans)

      # Create new robot state
      self.update_params()
    
      if self.theta1 > self.params['theta1'] + 0.01:
        self.theta1 -= 0.01
      if self.theta1 < self.params['theta1']:
        self.theta1 += 0.01

      if self.theta2> self.params['theta2'] + 0.01:
        self.theta2 -= 0.01
      if self.theta2 < self.params['theta2']:
        self.theta2 += 0.01

      if self.theta3 > self.params['theta3'] + 0.01:
        self.theta3 -= 0.01
      if self.theta3 < self.params['theta3']:
        self.theta3 += 0.01

    except KeyboardInterrupt:
          pass

def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
  node = StatePublisher()
  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
