import rclpy
from rclpy.node import Node

from interpolation_srv.srv import Jint

from geometry_msgs.msg import JointState

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Jint, 'jint_control_srv', self.jint_control_srv_callback)
   
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

        if request.interpolation_type != "linear" and request.interpolation_type != "spline":
            response.response = "Cannot perform linearization - wrong type of linearization"
            return response
        
        step_time = 0.1
        movement_time = response.movement_time
        
        
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