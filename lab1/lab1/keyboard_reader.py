
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray


from rcl_interfaces.msg import ParameterType
from rclpy.exceptions import ParameterNotDeclaredException

import curses

from os import system

import time

class KeyBoardReader(Node):

    def __init__(self):
        super().__init__('keyboard_reader')

        self.params = {}
        self.publisher_ = self.create_publisher(String, 'keyboard_reader' , 10)
        self.param_subscription = self.create_subscription(Float32MultiArray, 'param_topic', self.param_topic_callback, 10)
        self.param_subscription
        time.sleep(0.1)
        first_str = String()
        first_str.data = 'first'
        self.publisher_.publish(first_str)        
        self.stop = False
        first = True
        self.scr = Screen()
        self.print_control_info()
        self.scr.screen.refresh()
        # if first == True:

            # first = False

    def get_key(self):
        # while True:
        self.scr.assign_key()
        unicode_char = self.scr.key

        self.msg = String()         
        self.msg.data = chr(unicode_char)
        self.publish()
        self.scr.screen.refresh()
        if unicode_char==ord("q"):
            self.scr.screen.clear()
            self.scr.screen.refresh()
            curses.endwin()
            system("killall turtlesim_node")
            system("killall my_teleop")
            system("killall keyboard_reader")
    
    def print_control_info(self):
        X_CONSTANT = 10
        if bool(self.params):
            self.scr.screen.addstr(0, X_CONSTANT, "Control keys: ")
            self.scr.screen.addstr(1, X_CONSTANT, "UP:     " + self.params['przod'])
            self.scr.screen.addstr(2, X_CONSTANT, "DOWN:   " + self.params['tyl'])
            self.scr.screen.addstr(3, X_CONSTANT, "LEFT :  " + self.params['lewo'])
            self.scr.screen.addstr(4, X_CONSTANT, "RIGHT : " + self.params['prawo'])
            self.scr.screen.refresh()

    def print_speed(self):
        X_CONSTANT = 10
        self.scr.screen.addstr(6, X_CONSTANT, "Linear speed:  " + str(self.params['lin_vel']))
        self.scr.screen.addstr(7, X_CONSTANT, "Angular speed: " + str(self.params['ang_vel']))
        self.scr.screen.refresh()

    def publish(self):
        self.publisher_.publish(self.msg)

    def param_topic_callback(self, msg):
        print('')
        if msg.data[4] == 0.0 and msg.data[5]==0.0: 
            self.params["przod"] = chr(int(msg.data[0]))
            self.params["tyl"] = chr(int(msg.data[1]))
            self.params["lewo"] = chr(int(msg.data[2]))
            self.params["prawo"] = chr(int(msg.data[3]))
            self.params["lin_vel"] = (msg.data[4])
            self.params["ang_vel"] = (msg.data[5])
            self.print_control_info()
                  
        else:
            self.params["przod"] = chr(int(msg.data[0]))
            self.params["tyl"] = chr(int(msg.data[1]))
            self.params["lewo"] = chr(int(msg.data[2]))
            self.params["prawo"] = chr(int(msg.data[3]))
            self.params["lin_vel"] = (msg.data[4])
            self.params["ang_vel"] = (msg.data[5])
            self.print_speed()
        self.get_key()
        self.print_control_info()   
        
class Screen:
    def __init__(self):
        self.screen = curses.initscr()
        self.setup()

    def setup(self):
        curses.cbreak()        
        curses.noecho()
        self.screen.keypad(True)
        self.screen.refresh()

    def assign_key(self):
        self.key = self.screen.getch()

    def close_window(self):
        curses.endwin()
    
    def get_screen(self):
        return self.screen

def main(args=None):
    rclpy.init(args=args)

    keyboard_reader_node = KeyBoardReader()

    rclpy.spin(keyboard_reader_node)

    curses.endwin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()