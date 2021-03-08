
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterType
from rclpy.exceptions import ParameterNotDeclaredException

from geometry_msgs.msg import Twist

######################
import sys, termios, atexit
from select import select

# Save normal terminal settings
fd = sys.stdin.fileno()
old_term = termios.tcgetattr(fd)

def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# Debuffer new terminal until exit
new_term = termios.tcgetattr(fd)
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

atexit.register(set_normal_term)
set_curses_term()

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr != []

def getch():
    return sys.stdin.read(1)
################

class MyTeleop(Node):

    def __init__(self):
        super().__init__('my_teleop')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('przod', 'w')
        self.declare_parameter('tyl', 's')
        self.declare_parameter('lewo', 'a')
        self.declare_parameter('prawo', 'd')


        przod = rclpy.parameter.Parameter(
            'przod',
            rclpy.Parameter.Type.STRING,
            'w'
        )
        tyl = rclpy.parameter.Parameter(
            'tyl',
            rclpy.Parameter.Type.STRING,
            's'
        )
        lewo = rclpy.parameter.Parameter(
            'lewo',
            rclpy.Parameter.Type.STRING,
            'a'
        )
        prawo = rclpy.parameter.Parameter(
            'prawo',
            rclpy.Parameter.Type.STRING,
            'd'
        )
        all_new_parameters = [przod, tyl, lewo, prawo]
        self.set_parameters(all_new_parameters)
        
        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('string', None),
        #         ('string', None),
        #         ('string', None),
        #         ('string', None)
        #     ])

    # def timer_callback(self):
    #     przod = self.get_parameter('przod').get_parameter_value().string_value
    #     tyl = self.get_parameter('tyl').get_parameter_value().string_value
    #     lewo = self.get_parameter('lewo').get_parameter_value().string_value
    #     prawo = self.get_parameter('prawo').get_parameter_value().string_value

    #     # przod = rclpy.parameter.Parameter(
    #     #     'przod',
    #     #     rclpy.Parameter.Type.STRING,
    #     #     'w'
    #     # )
    #     # tyl = rclpy.parameter.Parameter(
    #     #     'tyl',
    #     #     rclpy.Parameter.Type.STRING,
    #     #     's'
    #     # )
    #     # lewo = rclpy.parameter.Parameter(
    #     #     'lewo',
    #     #     rclpy.Parameter.Type.STRING,
    #     #     'a'
    #     # )
    #     # prawo = rclpy.parameter.Parameter(
    #     #     'prawo',
    #     #     rclpy.Parameter.Type.STRING,
    #     #     'd'
    #     # )
    #     # all_new_parameters = [przod, tyl, lewo, prawo]
    #     # self.set_parameters(all_new_parameters)
    #     # self.set_parameters(all_params)

    #     #print("Sterowanie: " + przod " - przod , " + tyl " - tyl , " + lewo " - lewo , " + prawo " - prawo , q - wyjscie " )
    #     while True:
    #         if kbhit():
    #             key = getch()
    #             #handle_keypress(key)
    #             letter = key
    #             #letter = input("")
    #             if letter == przod:
    #                 lin_vel = 0.9
    #                 ang_vel = 0.0
    #             elif letter == tyl:
    #                 lin_vel = -0.9
    #                 ang_vel = 0.0
    #             elif letter == lewo:
    #                 lin_vel = 0.0
    #                 ang_vel = 0.9
    #             elif letter == prawo:
    #                 lin_vel = 0.0
    #                 ang_vel = -0.9
    #             elif letter == "q":
    #                 my_teleop.destroy_node()
    #                 rclpy.shutdown()
    #             else:
    #                 lin_vel = 0.0
    #                 ang_vel = 0.0
    #                 break
    #             msg = Twist()
    #             msg.linear.x = lin_vel
    #             msg.linear.y = 0.0
    #             msg.linear.z = 0.0
    #             msg.angular.x = 0.0
    #             msg.angular.y = 0.0
    #             msg.angular.z = ang_vel
    #             lin_vel = msg.linear.x
    #             ang_vel = msg.angular.x
    #             self.publisher_.publish(msg)
    #             break
    #                 #self.get_logger().info('\nPublishing: \nangular: %d\nlinear: %d' % (lin_vel, ang_vel))



def main(args=None):
    rclpy.init(args=args)

    my_teleop = MyTeleop()

    przod = my_teleop.get_parameter('przod').get_parameter_value().string_value
    tyl = my_teleop.get_parameter('tyl').get_parameter_value().string_value
    lewo = my_teleop.get_parameter('lewo').get_parameter_value().string_value
    prawo = my_teleop.get_parameter('prawo').get_parameter_value().string_value

    while True:
        if kbhit():
            key = getch()
            #handle_keypress(key)
            letter = key
            #letter = input("")
            if letter == przod:
                lin_vel = 0.9
                ang_vel = 0.0
            elif letter == tyl:
                lin_vel = -0.9
                ang_vel = 0.0
            elif letter == lewo:
                lin_vel = 0.0
                ang_vel = 0.9
            elif letter == prawo:
                lin_vel = 0.0
                ang_vel = -0.9
            elif letter == "q":
                my_teleop.destroy_node()
                rclpy.shutdown()
            else:
                lin_vel = 0.0
                ang_vel = 0.0
                break
            msg = Twist()
            msg.linear.x = lin_vel
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = ang_vel
            lin_vel = msg.linear.x
            ang_vel = msg.angular.x
            my_teleop.publisher_.publish(msg)
            

    rclpy.spin(my_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()