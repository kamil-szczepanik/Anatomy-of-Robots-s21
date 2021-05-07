import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Jint

from geometry_msgs.msg import JointState

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Jint, 'jint_control_srv', self.jint_control_srv_callback)

        self.declare_parameters(
                namespace='',
                parameters=[
                    ('position1', 0.0),
                    ('position2', 0.0),
                    ('position3', 0.0),
                ])
        self.position1 = self.get_parameter('position1').get_parameter_value().double_value
        self.position2 = self.get_parameter('position2').get_parameter_value().double_value
        self.position3 = self.get_parameter('position3').get_parameter_value().double_value
    
        self.movement_time = 0.0
        self.start_position1 = self.position1
        self.start_position2 = self.position2
        self.start_position3 = self.position3
   
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