from math import sin, cos, pi
import os
import rclpy
import yaml
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock
import pprint

# from PyKDL import *
from PyKDL import Chain, Joint, Frame, Rotation, Vector, Segment, ChainFkSolverPos_recursive, JntToCart, JntArray

class KDL_DKIN(Node):

    def __init__(self):
        super().__init__('KDL_DKIN')

        self.pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 10)
        


    def listener_callback(self, msg):

        xyz_rpy = XYZ_RPY().xyz_rpy
        
        # Lancuch kinematyczny
        chain = Chain()

        # Joint base_ext-arm
        base_ext_arm = Joint(Joint.RotZ)
        frame1 = Frame(
            Rotation.RPY(
                xyz_rpy['base_ext-arm']['roll'],
                xyz_rpy['base_ext-arm']['pitch'],
                xyz_rpy['base_ext-arm']['yaw'],
                ),
            Vector(
                xyz_rpy['base_ext-arm']['x'],
                xyz_rpy['base_ext-arm']['y'],
                xyz_rpy['base_ext-arm']['z'],
            )
        )
        segment1 = Segment(base_ext_arm, frame1)
        chain.addSegment(segment1)
        
        # Joint arm-hand
        arm_hand = Joint(Joint.RotZ)
        frame2 = Frame(
            Rotation.RPY(
                xyz_rpy['arm-hand']['roll'],
                xyz_rpy['arm-hand']['pitch'],
                xyz_rpy['arm-hand']['yaw']
                ),
            Vector(
                xyz_rpy['arm-hand']['x'],
                xyz_rpy['arm-hand']['y'],
                xyz_rpy['arm-hand']['z'],
            )
        )
        segment2 = Segment(arm_hand, frame2)
        chain.addSegment(segment2)

        # Joint hand-tool
        hand_tool = Joint(Joint.RotZ)
        frame3 = Frame(
            Rotation.RPY(
                xyz_rpy['hand-tool']['roll'],
                xyz_rpy['hand-tool']['pitch'],
                xyz_rpy['hand-tool']['yaw']
                ),
            Vector(
                xyz_rpy['hand-tool']['x'],
                xyz_rpy['hand-tool']['y'],
                xyz_rpy['hand-tool']['z'],
            )
        )
        segment3 = Segment(hand_tool, frame3)
        chain.addSegment(segment3)
        

            # Forward kinematics
        joint_positions = JntArray(3)
        joint_positions[0] = msg.position[0]
        joint_positions[1] = msg.position[1]
        joint_positions[2] = msg.position[2]

        # Solver
        fk = ChainFkSolverPos_recursive(chain)
        endFrame = Frame()
        fk.JntToCart(joint_positions,endFrame)

        qua = endFrame.M.GetQuaternion()
        tool_offset = Vector(
            0,
            0,
            0)
        endFrame.p + tool_offset
        xyz = endFrame.p
        print(xyz)


        qos_profile = QoSProfile(depth=10)
        

        pose = PoseStamped()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base"

        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation = Quaternion(x=qua[0], y=qua[1], z=qua[2], w=qua[3])

        self.pose_publisher.publish(pose)

class XYZ_RPY:
    def __init__(self):
        self.params = read_params()
        self.xyz_rpy = {}

        self.create_xyz_rpy()

    def create_base_el1(self):
        self.xyz_rpy['base-base_ext'] = {}
        self.xyz_rpy['base-base_ext']['x'] = self.params['base']['a']
        self.xyz_rpy['base-base_ext']['y'] = 0
        self.xyz_rpy['base-base_ext']['z'] =  self.params['base']['d']
        self.xyz_rpy['base-base_ext']['roll'] = self.params['base']['alpha']
        self.xyz_rpy['base-base_ext']['pitch'] = 0
        self.xyz_rpy['base-base_ext']['yaw'] = self.params['base']['theta']
    
    def crate_base_ext_arm(self):
        self.xyz_rpy['base_ext-arm'] = {}
        self.xyz_rpy['base_ext-arm']['x'] = self.params['base_ext']['a']
        self.xyz_rpy['base_ext-arm']['y'] = 0
        self.xyz_rpy['base_ext-arm']['z'] = self.params['base']['d']
        self.xyz_rpy['base_ext-arm']['roll'] = self.params['base_ext']['alpha']
        self.xyz_rpy['base_ext-arm']['pitch'] = 0
        self.xyz_rpy['base_ext-arm']['yaw'] = self.params['base_ext']['theta']

    def create_arm_hand(self):
        self.xyz_rpy['arm-hand'] = {}
        self.xyz_rpy['arm-hand']['x'] = self.params['arm']['a']
        self.xyz_rpy['arm-hand']['y'] = 0
        self.xyz_rpy['arm-hand']['z'] = self.params['arm']['d']
        self.xyz_rpy['arm-hand']['roll'] = self.params['arm']['alpha']
        self.xyz_rpy['arm-hand']['pitch'] = 0
        self.xyz_rpy['arm-hand']['yaw'] = self.params['arm']['theta']

    def create_hand_tool(self):
        self.xyz_rpy['hand-tool'] = {}
        self.xyz_rpy['hand-tool']['x'] = self.params['hand']['a']
        self.xyz_rpy['hand-tool']['y'] = 0
        self.xyz_rpy['hand-tool']['z'] = self.params['hand']['d']
        self.xyz_rpy['hand-tool']['roll'] = self.params['hand']['alpha']
        self.xyz_rpy['hand-tool']['pitch'] = 0
        self.xyz_rpy['hand-tool']['yaw'] = self.params['hand']['theta']

    def create_xyz_rpy(self):
        self.create_base_el1()
        self.crate_base_ext_arm()
        self.create_arm_hand()
        self.create_hand_tool()

def read_params():
    with open(os.path.join(get_package_share_directory('lab3'), 'params.yaml'), 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)

    return params


def main(args=None):
    rclpy.init(args=args)

    kdl = KDL_DKIN()
    try:
        rclpy.spin(kdl)
    except KeyboardInterrupt:
        kdl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()