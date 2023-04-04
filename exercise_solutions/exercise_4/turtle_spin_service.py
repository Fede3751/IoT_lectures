import rclpy
from rclpy.node import Node
from spin_interfaces.srv import Spin
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class TurtleSpinService(Node):

    def __init__(self):
        super().__init__('turtle_spin_service')

        self.spin_servce = self.create_service(Spin, 'spin_me_round', self.spin_callback)
        self.publish_to = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        
    def spin_callback(self, msg, response):
        
        dir = msg.dir
        dir = float(dir)

        move_msg = Twist()
        move_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z=5.0*dir)

        self.publish_to.publish(move_msg)
        response.res = "Spinning!"

        return response
    

def main():

    rclpy.init()

    spin_service = TurtleSpinService()

    rclpy.spin(spin_service)

    spin_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
