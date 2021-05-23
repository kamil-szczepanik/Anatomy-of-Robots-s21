import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import sys

from interpolation_srv.srv import OintXYZ

# Service - dwa parametry: czas w jakim robot ma się przemieścić i punkt w ktorym ma byc końcowka

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.client = self.create_client(OintXYZ, 'oint_control_srv')
        
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Oczekiwanie na serwis')
        self.request = OintXYZ.Request()

    def send_request(self):
        try:
            self.request.x = float(sys.argv[1])
            self.request.y = float(sys.argv[2])
            self.request.z = float(sys.argv[3])
            
            self.request.time = float(sys.argv[4])
            self.request.interpolation_type = sys.argv[5]

        except IndexError:
            print("Niepoprawna liczba parametrow")
            raise IndexError()
        except ValueError:
            print("Bledne parametry")
            raise ValueError()

        self.future = self.client.call_async(self.request)



def main(args=None):
    rclpy.init(args=args)

    try:
        client = MinimalClientAsync()
        client.send_request()
    except Exception as e:
        print("Nie udalo sie uruchomic klienta")
        print(e)
    else:
        try:
            while rclpy.ok():
                rclpy.spin_once(client)
                if client.future.done():
                    try:
                        response = client.future.result()
                    except Exception as e:
                        client.get_logger().error("Interpolacja nieudana: " + str(e))
                    else:
                        client.get_logger().info(response.response)
                        return
        except KeyboardInterrupt:
            client.get_logger().warning("Nie otrzymano informacji zwrotnej")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()