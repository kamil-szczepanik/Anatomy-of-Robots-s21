
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray

import curses

import time


class KeyBoardReader(Node):

    def __init__(self):
        super().__init__('keyboard_reader')
        self.turtle = '/turtle1/'
        self.publisher_ = self.create_publisher(String, 'keyboard_reader' , 10)
        self.param_subscription = self.create_subscription(Float32MultiArray, 'param_topic', self.param_topic_callback, 10)
        self.stop = False
        self.scr = Screen()
        self.get_key()
        self.parameters = {}
        self.print_control_info()
        self.scr.screen.refresh()

    def get_key(self):
        while True:
            self.scr.assign_key()
            unicode_char = self.scr.key
            self.msg = String()         
            self.msg.data = chr(unicode_char)
            self.publish()
            self.scr.screen.refresh()
    
    def print_control_info(self):
        X_CONSTANT = 30
        self.scr.screen.addnstr(0, X_CONSTANT, "Control keys: ")
        self.scr.screen.addnstr(1, X_CONSTANT, "UP: " + self.parameters['przod'])
        self.scr.screen.addnstr(2, X_CONSTANT, "DOWN: "+ self.parameters['tyl'])
        self.scr.screen.addnstr(3, X_CONSTANT, "LEFT : "+ self.parameters['lewo'])
        self.scr.screen.addnstr(4, X_CONSTANT, "RIGHT : "+ self.parameters['prawo'])
        self.scr.screen.refresh()

    def publish(self):
        self.publisher_.publish(self.msg)

    def param_topic_callback(self, msg):
        self.parameters["przod"] = msg.data[0]
        self.parameters["tyl"] = msg.data[1]
        self.parameters["lewo"] = msg.data[2]
        self.parameters["prawo"] = msg.data[3]

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