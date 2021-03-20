
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray

import curses

import time


class KeyBoardReader(Node):

    def __init__(self):
        super().__init__('keyboard_reader')
        self.parameters = {}      
        self.publisher_ = self.create_publisher(String, 'keyboard_reader' , 10)
        self.param_subscription = self.create_subscription(Float32MultiArray, 'param_topic', self.param_topic_callback, 10)
        self.param_subscription
        self.stop = False
        first = True
        self.scr = Screen()
        self.print_control_info()
        self.scr.screen.refresh()
        if first == True:
            first_str = String()
            first_str.data = 'first'
            self.publisher_.publish(first_str)
            first = False




    def get_key(self):
        # while True:
        self.scr.assign_key()
        unicode_char = self.scr.key

        self.msg = String()         
        self.msg.data = chr(unicode_char)
        self.publish()
        self.scr.screen.refresh()
        if unicode_char==ord("q"):
            curses.endwin()      
    
    def print_control_info(self):
        X_CONSTANT = 10
        if bool(self.parameters):
            self.scr.screen.addstr(0, X_CONSTANT, "Control keys: ")
            self.scr.screen.addstr(1, X_CONSTANT, "UP:     " + self.parameters['przod'])
            self.scr.screen.addstr(2, X_CONSTANT, "DOWN:   " + self.parameters['tyl'])
            self.scr.screen.addstr(3, X_CONSTANT, "LEFT :  " + self.parameters['lewo'])
            self.scr.screen.addstr(4, X_CONSTANT, "RIGHT : " + self.parameters['prawo'])
            self.scr.screen.refresh()

        
    def print_speed(self):
        X_CONSTANT = 10
        self.scr.screen.addstr(6, X_CONSTANT, "Linear speed:  " + str(self.parameters['lin_vel']))
        self.scr.screen.addstr(7, X_CONSTANT, "Angular speed: " + str(self.parameters['ang_vel']))
        self.scr.screen.refresh()


    def publish(self):
        self.publisher_.publish(self.msg)

    def param_topic_callback(self, msg):
        print('')
        if msg.data[4] == 0.0 and msg.data[5]==0.0: 
            self.parameters["przod"] = chr(int(msg.data[0]))
            self.parameters["tyl"] = chr(int(msg.data[1]))
            self.parameters["lewo"] = chr(int(msg.data[2]))
            self.parameters["prawo"] = chr(int(msg.data[3]))
            self.parameters["lin_vel"] = (msg.data[4])
            self.parameters["ang_vel"] = (msg.data[5])
            self.print_control_info()
                  
        else:
            self.parameters["przod"] = chr(int(msg.data[0]))
            self.parameters["tyl"] = chr(int(msg.data[1]))
            self.parameters["lewo"] = chr(int(msg.data[2]))
            self.parameters["prawo"] = chr(int(msg.data[3]))
            self.parameters["lin_vel"] = (msg.data[4])
            self.parameters["ang_vel"] = (msg.data[5])
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
    my_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()