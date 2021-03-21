
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterType
from rclpy.exceptions import ParameterNotDeclaredException

from geometry_msgs.msg import Twist

from std_msgs.msg import String, Float32MultiArray

import curses

import time


class MyTeleop(Node):

    def __init__(self):
        super().__init__('my_teleop')
        self.turtle = '/turtle1/'
        self.topic = self.turtle + 'cmd_vel'
        self.publisher_ = self.create_publisher(Twist, self.topic , 10)
        self.param_publisher_ = self.create_publisher(Float32MultiArray, 'param_topic', 10)          
        self.subscription = self.create_subscription(String, 'keyboard_reader', self.listener_callback, 10)
        self.first = True
        self.subscription
        self.setup_params()
        self.previous_letter = None

    def setup_params(self):
        self.parameters = {}
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('przod', 'w'),
                    ('tyl', 's'),
                    ('lewo', 'a'),
                    ('prawo', 'd')
                ])
        self.parameters["przod"] = self.get_parameter("przod")._value
        self.parameters["tyl"] = self.get_parameter("tyl")._value
        self.parameters["lewo"] = self.get_parameter("lewo")._value
        self.parameters["prawo"] = self.get_parameter("prawo")._value

    def listener_callback(self, msg):
        if self.first == True:
            param_msg = self.get_param_message()
            self.param_publisher_.publish(param_msg)
            self.first = False
        else:
            self.letter = msg.data
            self.get_logger().info('I heard: "%s"' % msg.data)
            self.publish()
            param_msg = self.get_param_message()
            self.param_publisher_.publish(param_msg)
            self.previous_letter = msg.data

        
    def get_velocity_from_letter(self):

        if self.previous_letter is not None and self.previous_letter==self.letter:
            gain = 1.2
        else:
            gain = 1

        self.parameters["przod"] = self.get_parameter("przod")._value
        self.parameters["tyl"] = self.get_parameter("tyl")._value
        self.parameters["lewo"] = self.get_parameter("lewo")._value
        self.parameters["prawo"] = self.get_parameter("prawo")._value
        przod = self.parameters["przod"]
        tyl = self.parameters["tyl"]
        lewo = self.parameters["lewo"]
        prawo = self.parameters["prawo"]      

        if self.letter == przod:
            lin_vel = 1.0
            ang_vel = 0.0
            return lin_vel, ang_vel

        elif self.letter == tyl:
            lin_vel = -1.0
            ang_vel = 0.0
            return lin_vel, ang_vel

        elif self.letter == lewo:
            lin_vel = 0.0
            ang_vel = 1.0
            return lin_vel, ang_vel

        elif self.letter == prawo:
            lin_vel = 0.0
            ang_vel = -1.0
            return lin_vel, ang_vel

        elif self.letter == "q":
            self.stop = True
            rclpy.spin(self)
            self.destroy_node()
            rclpy.shutdown()

        else:
            lin_vel = 0.0
            ang_vel = 0.0    
            return lin_vel, ang_vel      

    def get_message_to_publish(self):
        lin_vel, ang_vel = self.get_velocity_from_letter()
        msg = Twist()
        msg.linear.x = lin_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = ang_vel
        return msg

    def publish(self):
        msg = self.get_message_to_publish()
        self.publisher_.publish(msg)

    def get_param_message(self):
        msg = Float32MultiArray()
        if self.first == True:
            msg.data = [float(ord(self.parameters['przod'])), float(ord(self.parameters['tyl'])), float(ord(self.parameters['lewo'])), float(ord(self.parameters['prawo'])), 0.0 , 0.0 ]
        elif bool(self.parameters):
            lin_vel, ang_vel = self.get_velocity_from_letter()
            msg.data = [float(ord(self.parameters['przod'])), float(ord(self.parameters['tyl'])), float(ord(self.parameters['lewo'])), float(ord(self.parameters['prawo'])), lin_vel, ang_vel]
        return msg


def main(args=None):
    rclpy.init(args=args)

    my_teleop = MyTeleop()
    rclpy.spin(my_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()