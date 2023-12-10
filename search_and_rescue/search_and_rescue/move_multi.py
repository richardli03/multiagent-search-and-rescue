import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
from numpy import pi
from .constants import Names


class MultiMover(Node):
    """Defining an interface class that makes it easy and intuitive to move multiple robots

    :param Node: _description_
    :type Node: _type_
    """

    def __init__(self):
        super().__init__("move_multi")
        self.ally_vel = self.create_publisher(Twist, "ally/cmd_vel", 10)
        self.billy_vel = self.create_publisher(Twist, "billy/cmd_vel", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)

    def move(self, lin: float, ang: float, robot_name: Names):
        """Move the robot with some linear velocity and some angular velocity.

        Automatically converts the input to a float(as the Twist() message
        only accepts floats), then publishes a message (hopefully to /cmd_vel)
        that will move the robot accordingly.

        :param lin: the linear velocity to give the robot
        :type lin: float
        :param ang: the angular velocity to give the robot
        :type ang: float
        """
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        if robot_name == Names.ALLY:
            self.ally_vel.publish(msg)
        elif robot_name == Names.BILLY:
            self.billy_vel.publish(msg)
        else:
            raise Exception(f"name {robot_name} isn't valid!")

    def turn_left(self, robot_name: Names, angular_vel=0.3):
        """Turn a specified robot some number of degrees to the left.

        :param robot_name: the name of the robot to drive
        :type robot_name: Names
        :param angle: angle in degrees
        :type angle: int
        """
        self.move(0.0, angular_vel, robot_name)
        # self.move(0.0, 0.0, robot_name)

    def turn_right(self, robot_name: Names, angular_vel=-0.3):
        """Turn a specified robot some number of degrees to the right.

        :param robot_name: the name of the robot to drive
        :type robot_name: Names
        :param angle: angle in degrees
        :type angle: int
        """
        self.move(0.0, angular_vel, robot_name)

    def drive_forward(self, robot_name: Names, linear_vel=0.2):
        """Drive a specified robot forward

        :param robot_name: the name of the robot to drive
        :type robot_name: Names
        :param linear_vel: the speed at which to drive, defaults to 0.2
        :type linear_vel: float, optional
        """
        self.move(linear_vel, 0.0, robot_name)

    def stop(self, robot_name: Names):
        """Stop a specified robot

        :param robot_name: the name of the robot to stop
        :type robot_name: Names
        """
        self.move(0.0, 0.0, robot_name)

    def run_loop(self):
        i = 0
        while i < 100:
            self.turn_left(Names.ALLY)
            self.turn_left(Names.BILLY)
            sleep(0.1)

        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = MultiMover()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()
