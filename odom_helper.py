"""
Here, we have some helper functions defined for converting poses from odom subscription for each neato.
This code is NOT written by us, and is directly taken from the robot localization project, written by Paul Ruvolo.

Github link: https://github.com/comprobo23/robot_localization 
"""

from geometry_msgs.msg import Pose
import math

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
