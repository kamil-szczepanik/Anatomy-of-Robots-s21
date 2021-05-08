import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from interpolation_srv.srv import Oint
# # desired positions of joint
# float64 x 
# float64 y
# float64 z
# float64 roll
# float64 pitch
# float64 yaw

# # type of interpolation
# string interpolation_type

# # movement time
# float64 time
# ---
# # service response
# string response

from geometry_msgs.msg import JointState

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Oint, 'oint_control_srv', self.oint_control_srv_callback)
   
    def jint_control_srv_callback(self, request, response):

        self.get_logger().info('Incoming request\n'+
                                ' - position1: %d\n' +
                                ' - position2: %d\n'+
                                ' - position3: %d\n'+
                                ' -- time: %d\n'+
                                ' --- interpolation type:%d' % (request.position1, 
                                                                request.position2, 
                                                                request.position3, 
                                                                request.time, 
                                                                request.interpolation))

        self.rate = 0.1
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.orient_roll = 0
        self.orient_pitch = 0
        self.orient_yaw = 0
        

        position = PoseStamped()
        moves = (int)(request.time/self.rate)

        if request.interpolation_type == "Linear":
            self.linear_interpolation(pose, tf, movement_time)
        
    def linear_interpolation(q0, qf, tf, movement_time):
        return q0 + ((qf-q0)/tf)*movement_time

    def spline_interpolation():
        pass



def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()