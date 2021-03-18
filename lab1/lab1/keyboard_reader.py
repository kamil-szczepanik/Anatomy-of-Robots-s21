
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import curses

import time


class KeyBoardReader(Node):

    def __init__(self):
        super().__init__('keyboard_reader')
        self.turtle = '/turtle1/'
        self.publisher_ = self.create_publisher(String, 'keyboard_reader' , 10)
        self.stop = False
        self.scr = Screen()
        # self.key = String
        self.get_key()

    def get_key(self):
        while True:
            self.scr.assign_key()
            unicode_char = self.scr.key
            self.msg = String()         
            self.msg.data = chr(unicode_char)
            self.publish()
            

    def publish(self):
            self.publisher_.publish(self.msg)



class Screen:
    def __init__(self):
        self.screen = curses.initscr()
        self.setup()

    def setup(self):
        curses.cbreak()        
        curses.noecho()
        self.screen.keypad(True)
        # print parameters
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
    
    # curses.echo()
    # curses.nocbreak()
    # my_teleop.screen.keypad(False)
    


    curses.endwin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()