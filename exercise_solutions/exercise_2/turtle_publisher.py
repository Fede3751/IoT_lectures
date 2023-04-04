import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


turtle_moves = [
    Vector3(x=1.0, y=0.0, z=0.0),
    Vector3(x=0.0, y=1.0, z=0.0),
    Vector3(x=-1.0, y=0.0, z=0.0),
    Vector3(x=0.0, y=-1.0, z=0.0)
]


class TurtlePublisher(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer = self.create_timer(1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear = turtle_moves[self.i%4]
        msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.i += 1
        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args = args)

    turtle_publisher = TurtlePublisher()

    rclpy.spin(turtle_publisher)

    turtle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
