"""
Here, we have some helper functions defined for converting poses from odom subscription for each neato.
This code is NOT written by us, and is directly taken from the robot localization project, written by Paul Ruvolo.

Github link: https://github.com/comprobo23/robot_localization 
"""

from geometry_msgs.msg import Pose
import math

    
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

def convert_pose_to_xy_and_theta(pose: Pose):
        """Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple"""
        orientation_tuple = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        angles = euler_from_quaternion(*orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] 
    CODE FROM https://github.com/comprobo23/robot_localization/blob/d424bff3cd1c3ce12f97aa1e346d6b9866394cdc/robot_localization/angle_helpers.py
    """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ 
    Calculates the difference between angle a and angle b (both should
    be in radians) the difference is always based on the closest
    rotation from angle a to angle b.
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    CODE FROM https://github.com/comprobo23/robot_localization/blob/d424bff3cd1c3ce12f97aa1e346d6b9866394cdc/robot_localization/angle_helpers.py
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return (d1)
    else:
        return (d2)

def distance_diff(a,b):
    return abs(a-b)
