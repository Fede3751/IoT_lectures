import math

import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

from turtlesim.msg import Pose

from patrol_interfaces.action import PatrolCommandInterface



class PatrolCommand(Node):

    def __init__(self):
        
        super().__init__('patrol_command')

        self.turtle_position = (0,0)
        self.turtle_rotation = 0


        # Subscription to the pose topic. We do this to keep listenting
        # for the turtle position and store its position
        self.pose_subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.register_turtle_position,
            10
        )

        # Publisher for cmd_vel. We publish here movement msgs
        self.cmd_publisher = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10
        )
        
        # Action server for the Patrolling Action. We will receive our
        # goal here. ANd handle it with the method patrol_callback(...)
        self.action_command = ActionServer(
            self,
            PatrolCommandInterface,
            'command_turtle',
            self.patrol_callback
        )



    # Pose callback. Simply stores position and rotation on two different
    # fieds for simpler use
    def register_turtle_position(self, pose_msg : Pose):

        self.turtle_position = (pose_msg.x, pose_msg.y)
        self.turtle_rotation = pose_msg.theta


    # Patrol callback. Used to execute the whole action
    def patrol_callback(self, patrol_goal : PatrolCommandInterface.Goal):
        
        self.get_logger().info('Patrol command received')
        feedback = PatrolCommandInterface.Feedback()
    
        # Store the targets here
        targets = patrol_goal.request.targets
        
        target_count = 0
        for target in targets:
            next_target = (target.x, target.y)

            # For each target we first rotate, and then move towards it
            self.rotate_to_target(next_target)
            self.move_to_target(next_target)

            # When a point is reached, we send a feedback message
            target_count += 1
            self.get_logger().info("Target %d reached. Sending feedback" % target_count)
            feedback.feedback_msg = "Target %d reached" % target_count
            patrol_goal.publish_feedback(feedback)


        self.get_logger().info("Action completed! Sending result message")

        # And then another final message when it is completed
        patrol_goal.succeed()
        patrol_result = PatrolCommandInterface.Result()
        patrol_result.result_msg = "Patrolling complete!"
        return patrol_result


    # Function used to rotate the turtle and face the next target
    def rotate_to_target(self, target, eps = 0.05):
        
        # We compute the angle between the current target position and the target
        # position here
        target_angle = angle_between_points(self.turtle_position, target)
        angle_to_rotate = target_angle - self.turtle_rotation

        # We verify the optimal direction of the rotation here
        rotation_dir = 1
        if angle_to_rotate < 0 or angle_to_rotate > math.pi:
            rotation_dir = -1
        
        
        # Prepare the cmd_vel message
        move_msg = Twist()
        move_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z=1.0 * rotation_dir)

        # Publish the message until the correct rotation is reached (accounting for some eps error)
        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.turtle_rotation
            self.cmd_publisher.publish(move_msg)

        # When done, send a stop message to be sure that the turtle doesn't
        # overshoot its target
        stop_msg = Twist()
        stop_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_publisher.publish(stop_msg)


    # Function used to move the turtle towards its next target
    def move_to_target(self, target, eps = 0.1, offset_eps = 0.01):

        # We instantiate a simple move forward message here
        move_msg = Twist()
        move_msg.linear = Vector3(x=1.0, y=0.0, z=0.0)

        # Until the target is reached, send the message...
        while point_distance(self.turtle_position, target) > eps:

            # Here, we additionally account for some rounding and epsilon error that
            # may have happened during the rotation. If during the movement the turtle
            # is not properly facing the target anymore, we rotate it by the correct
            # amount to send it to the right direction.

            move_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
            target_angle = angle_between_points(self.turtle_position, target)
            angle_offset = target_angle - self.turtle_rotation

            if abs(angle_offset) > offset_eps:
                move_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_offset))
            
            self.cmd_publisher.publish(move_msg)

        
        # Stop message
        stop_msg = Twist()
        stop_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_publisher.publish(stop_msg)




#----------- Math functions. Placed here to have one file solution --------------

def angle_between_points(p0 : tuple, p1 : tuple):

    '''
    Computes the angle between the two given points, in radiants.
    p0: the coordinates of the first point (x0, y0)
    p1: the coordinates of the second point (x1, y1)
    '''

    vector_between = (p1[0] - p0[0], p1[1] - p0[1])

    norm = math.sqrt(vector_between[0] ** 2 + vector_between[1] ** 2)
    direction = (vector_between[0] / norm, vector_between[1] / norm)

    # x and y are used inverted here, as the x vector moves the turtle forward, and not sideways
    return math.atan2(direction[1], direction[0])

def point_distance(p0 : tuple, p1 : tuple):

    '''
    Computes the distance between the two given points.
    p0: the coordinates of the first point (x0, y0)
    p1: the coordinates of the second point (x1, y1)
    '''

    vector_between = (p1[0] - p0[0], p1[1] - p0[1])
    return math.sqrt(vector_between[0] ** 2 + vector_between[1] ** 2)


#--------------------------------------------------------------------------------


def main():

    rclpy.init()

    executor = MultiThreadedExecutor()
    patrol_command = PatrolCommand()

    executor.add_node(patrol_command)
    executor.spin()

    executor.shutdown()
    patrol_command.destroy_node()


    rclpy.shutdown()
