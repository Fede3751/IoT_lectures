import math


#   Math utilities for Exercise 5.
#   These functions should come in hand if you are having problems in moving the Turtle in the right direction.
#   Depending on how you are approaching the solution, you may need only some of the functions provided here.
#
#   Regarding the project structure, your node should be running using a MultiThreadedExecutor just like advised.
#   It should listen for the current turtle position by listening to the topic /turtle1/pose, and store that pose
#   on a local attribute.
#   Moving the turtle after that should just consist in sending the right directions to /turtle1/cmd_vel, based on the
#   current turtle position, and the position of the next target.
#
#   Good luck!




#    This function can be used to compute the angle between points, in radiants.
#    It can be useful if you want to rotate the turtle to the right angle before moving it (or while you are moving it).
#    Rotation of the Turtle is given by the interface message turtlesim/msg/Pose.theta, in radiants.

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



#    The following function computes the unit vector between two given points.
#    The unit vector is defined by the vector between the two points, divided by its norm.
#    By multiplying both resulting components by a scalar value, you can have a constant speed applied to your movement.
#    This function shoudln't be used if you plan to rotate your turtle, but it is good to know how to do something
#    like this, if required.

def unit_vector_between_points(p0 : tuple, p1 : tuple):
	
    '''
    Computes the unit vector between the two given points.
    Direction of the vector is from p0 to p1.
    p0: the coordinates of the first point (x0, y0)
    p1: the coordinates of the second point (x1, y1)
    '''

    vector_between = (p1[0] - p0[0], p1[1] - p0[1])

    norm = math.sqrt(vector_between[0] ** 2 + vector_between[1] ** 2)
    direction = (vector_between[0] / norm, vector_between[1] / norm)

    return (direction[0], direction[1])


#   This function computes the distance between two given points.
#   It should be used to check if the turtle has reached its objective.
#   Note that reaching the exact target point is close to impossible, as we are working with floating numbers.
#   You should have a margin of error epsilon applied to your distance check, or the turtle will always miss its target.

def point_distance(p0 : tuple, p1 : tuple):

    '''
    Computes the distance between the two given points.
    p0: the coordinates of the first point (x0, y0)
    p1: the coordinates of the second point (x1, y1)
    '''

    vector_between = (p1[0] - p0[0], p1[1] - p0[1])
    return math.sqrt(vector_between[0] ** 2 + vector_between[1] ** 2)

