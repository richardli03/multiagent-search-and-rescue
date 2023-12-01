import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
from numpy import pi
from .constants import Names


class Multimove(Node):
    def __init__(self):
        super().__init__("move_multi")
        self.publisher = self.create_publisher(String, "greetings", 10)
        self.ally_vel = self.create_publisher(Twist, "ally/cmd_vel", 10)
        self.billy_vel = self.create_publisher(Twist, "billy/cmd_vel", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)

    def move(self, lin, ang, robot_name: Names):
        """Move the robot with some linear velocity and some angular velocity.

        Automatically converts the input to a float(as the Twist() message
        only accepts floats), then publishes a message (hopefully to /cmd_vel)
        that will move the robot accordingly.

        :param lin: the linear velocity to give the robot
        :type lin: int
        :param ang: _description_
        :type ang: _type_
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
        """Turn the robot some number of degrees to the left.

        :param angle: angle in degrees
        :type angle: int
        """
        self.move(0.0, angular_vel, robot_name)
        # self.move(0.0, 0.0, robot_name)

    def run_loop(self):
        self.turn_left(Names.ALLY)
        self.turn_left(Names.BILLY)

        self.move(0, 0, Names.ALLY)
        self.move(0, 0, Names.BILLY)

        sleep(3)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = Multimove()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()
