import time

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from ros_gz_interfaces.srv import ControlWorld


class VehicleController(Node):

    def __init__(self):
        super().__init__('command_vehicle')


        # Subscribe to the World Control service to pause and unpause the
        # simulation
        self.simulation_control = self.create_client(
            ControlWorld,
            '/world/vehicle_blue_world/control'
        )

        # Usual cmd_vel topic, used to publish movement data
        self.cmd_vel_topic = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )


        # Play the simulation
        self.start_simulation()

        # Give some time to Gazebo to kick in
        time.sleep(3)

        # Start perfroming the "square"
        self.move_square()

        # Stop the simulation
        self.stop_simulation()

    
    def start_simulation(self):

        self.get_logger().info("Starting simulation...")

        while not self.simulation_control.service_is_ready():
            pass

        req_message =  ControlWorld.Request()
        req_message.world_control.pause = False

        self.simulation_control.call_async(req_message)


    def stop_simulation(self):

        while not self.simulation_control.service_is_ready():
            pass

        req_message =  ControlWorld.Request()
        req_message.world_control.pause = True

        self.simulation_control.call_async(req_message)


    def move_square(self):

        #
        # Simple function to move the vehicle in roughly a square.
        # For a "perfect" square you shoud listen to the odometry topic and rotate
        # the vehicle accordingly, but it is beyond the scope of this exercise.
        # Note that if the simulation is not running at full speed, the resulting
        # movement will look way off from a square. We're not synchronizing with the
        # clock of the simulation.
        #
        # The function simply iterates over two movements (forward and rotate) for
        # each edge/corner of the "square". It then waits some time for the movement
        # to be completed before publishing the next one.
        #

        movements_linear = [
            Vector3(x = 2.0, y = 0.0, z = 0.0),
            Vector3(x = 0.0, y = 0.0, z = 0.0)
        ]
        movements_angular = [
            Vector3(x = 0.0, y = 0.0, z = 0.0),
            Vector3(x = 0.0, y = 0.0, z = 1.15)
        ]

        for i in range(9):
            mov_cmd = Twist(linear = movements_linear[i%2], angular = movements_angular[i%2])
            self.cmd_vel_topic.publish(mov_cmd)
            time.sleep(1.5)


        stop_mov = Twist()
        stop_mov.linear = Vector3(x = 0.0, y = 0.0, z = 0.0)
        stop_mov.angular = Vector3(x = 0.0, y = 0.0, z = 0.0)
        self.cmd_vel_topic.publish(stop_mov)

        time.sleep(1.5)





def main():

    rclpy.init()

    vehicle_controller = VehicleController()
    executor = MultiThreadedExecutor()

    executor.add_node(vehicle_controller)
    executor.spin()

    executor.shutdown()
    vehicle_controller.destroy_node()

    rclpy.shutdowon()



if __name__ == '__main__':
    main()
