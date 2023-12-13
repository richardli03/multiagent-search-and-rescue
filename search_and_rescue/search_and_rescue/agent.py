import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32MultiArray
from search_and_rescue.odom_helper import (
    euler_from_quaternion,
    angle_diff,
    distance_diff,
)
from enum import Enum
import math
import time
from nav_msgs.msg import Odometry
from .constants import Names

NEATO_LENGTH = 0.5


class State(Enum):
    MOVE_STRAIGHT_X_RIGHT = "move straight in x direction right"
    MOVE_STRAIGHT_X_LEFT = "move straight in x direction left"
    MOVE_STRAIGHT_Y_LEFT = "move straight in y direction wiht next turn left"
    MOVE_STRAIGHT_Y_RIGHT = "move straight in y direction wiht next turn right"
    TURN_LEFT_2 = "Turn 90 degrees left 2"
    TURN_RIGHT_2 = "Turn 90 degrees right 2"
    TURN_LEFT_1 = "Turn 90 degrees left 1"
    TURN_RIGHT_1 = "Turn 90 degrees right 1"
    FOUND_OBJECT = "Found object"
    COMPLETE_SEARCH = "Completed area search"


class Agent(Node):
    def __init__(self):
        super().__init__("agent")
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.process_odom, 10
        )
        self.move = Twist()
        self.odom = None
        self.state = State.MOVE_STRAIGHT_X_RIGHT
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.brain_sub = self.create_subscription(
            UInt32MultiArray, "map_path", self.parse_map_bounds, 10
        )
        self.init_x = 0
        self.init_y = 0
        self.current_x = 0
        self.current_y = 0
        self.next_y = 0
        self.max_x = 2
        self.max_y = 5
        self.linear_speed = 0.3  # CHANGE VALUE
        self.angular_speed = 0.3
        self.current_angle = 0
        self.next_angle = -1 * math.pi / 2
        ## orienting robot in direction of most weighted/closest objects
        timer_period = 1
        self.threshold = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.object_x = 3
        self.object_y = 20
        self.distance_diff = self.max_x
        self.angle_diff = math.pi / 2

    def process_odom(self, msg: Odometry):
        """
        subscribes to odom topic and sets robot's own odom to a variable in node.

        Args:
            - odom: A message of type Odometry
        """
        self.odom = msg

    def parse_map_bounds(self, msg: UInt32MultiArray):
        """
        A callback function that , given messages from the brain of what neato's bounds are via topic, parses the data and save it.

        Args:
            - msg: A message type UInt32MultiArray and 2 integer array from topic "WHAT IS TOPIC NAME" that gives the coordinates of
            neato's bounds in odom frame.
        """
        self.max_x = msg.data[0]
        self.max_y = msg.data[1]

    def move_straight(self):
        """
        When called, will make the robot drive straight by publishing to cmd_vel.
        """
        self.move.angular.z = 0.0
        self.move.linear.x = self.linear_speed * self.distance_diff
        self.publisher.publish(self.move)

    def turn(self):
        """
        When called, will make the robot turn based on angle difference.
        """
        self.move.angular.z = self.angular_speed * self.angle_diff * -1
        self.move.linear.x = 0.0
        self.publisher.publish(self.move)

    def check_object(self):
        """
        Checks if the object/person to be found is in the current area.
        """
        if self.object_x == self.current_x and self.object_y == self.current_y:
            return True
        return False

    def find_object(self):
        """
        Publish message to main brain indicating that a person is found.
        """
        self.move.angular.z = 0.0
        self.move.linear.x = 0.0
        self.publisher.publish(self.move)
        print(
            "Send coordinate to main brain, activating other robots to move to localization"
        )

    def complete_search(self):
        """
        Publishes message to main brain that robot has completed the search in the area assigned.
        """
        self.move.angular.z = 0.0
        self.move.linear.x = 0.0
        self.publisher.publish(self.move)
        print("Send message that area assigned is searched")

    def choose_state(self):
        """
        State machine function that chooses the robot's "state" of movement based on its position on the map/area it is searching, and its previous state.
        """
        # Final States
        if self.check_object():
            self.state = State.FOUND_OBJECT
        if self.next_y >= self.max_y:
            self.state = State.COMPLETE_SEARCH

        # Search Loop
        if (
            self.state == State.MOVE_STRAIGHT_X_RIGHT
            and self.distance_diff < self.threshold
        ):
            print(self.distance_diff)
            print("SWITCH TO TURN LEFT 1")
            self.state = State.TURN_LEFT_1
            self.next_angle = self.current_angle + math.pi / 2
            print(f"Next angle: {self.next_angle * 180 / math.pi}")

        elif self.state == State.TURN_LEFT_1 and abs(self.angle_diff) < self.threshold:
            print("SWITCH TO TURN MOVE STRAIGHT Y")
            self.state = State.MOVE_STRAIGHT_Y_LEFT
            self.next_y = self.current_y + NEATO_LENGTH

        elif (
            self.state == State.MOVE_STRAIGHT_Y_LEFT
            and self.distance_diff < self.threshold
        ):
            print("SWITCH TURN LEFT 2")
            self.state = State.TURN_LEFT_2
            self.next_angle = self.current_angle + math.pi / 2
            print(f"Next angle: {self.next_angle * 180 / math.pi}")

        elif self.state == State.TURN_LEFT_2 and abs(self.angle_diff) < self.threshold:
            print("SWITCH TURN MOVE STRAIGHT X LEFT ")
            print(f"Current Angle: {self.current_angle} --SWITCH")
            self.state = State.MOVE_STRAIGHT_X_LEFT
        elif (
            self.state == State.MOVE_STRAIGHT_X_LEFT
            and self.distance_diff < self.threshold
        ):
            self.state = State.TURN_RIGHT_1
            self.next_angle = self.current_angle - math.pi / 2
            print(f"Next angle: {self.next_angle * 180 / math.pi}")

        elif self.state == State.TURN_RIGHT_1 and abs(self.angle_diff) < self.threshold:
            self.state = State.MOVE_STRAIGHT_Y_RIGHT
            self.next_y = self.current_y + NEATO_LENGTH

        elif (
            self.state == State.MOVE_STRAIGHT_Y_RIGHT
            and self.distance_diff < self.threshold
        ):
            self.state = State.TURN_RIGHT_2
            self.next_angle = self.current_angle - math.pi / 2
            print(f"Next angle: {self.next_angle * 180 / math.pi}")

        elif self.state == State.TURN_RIGHT_2 and abs(self.angle_diff) < self.threshold:
            self.state = State.MOVE_STRAIGHT_X_RIGHT

    def run_loop(self):
        """
        Function that loops the behaviors of the robot, aka the "lawnmover" behavior. Identifying state of robot, and moving it accordingly.
        """
        # re-getting odom pose from odom topic, and saving as the new pose
        new_pose: Odometry = self.odom
        if new_pose is None:
            return

        # extracting current x and y from odom pose
        self.current_x = new_pose.pose.pose.position.x
        self.current_y = new_pose.pose.pose.position.y
        print(f"pos: {self.current_x}, {self.current_y}\n")
        _, _, yaw_z = euler_from_quaternion(
            new_pose.pose.pose.orientation.x,
            new_pose.pose.pose.orientation.y,
            new_pose.pose.pose.orientation.z,
            new_pose.pose.pose.orientation.w,
        )
        self.current_angle = yaw_z
        print(f"angle: {self.current_angle * 180 / math.pi}")

        # calling choose state to get robot movement started, then moving robot accordingly.
        self.choose_state()
        print(self.state)
        match self.state:
            case State.MOVE_STRAIGHT_X_LEFT:
                self.distance_diff = distance_diff(self.current_x, self.init_x)
                print(f"distance diff: {self.distance_diff}")
                self.move_straight()
            case State.MOVE_STRAIGHT_X_RIGHT:
                self.distance_diff = distance_diff(self.current_x, self.max_x)
                print(f"distance diff: {self.distance_diff}")
                self.move_straight()
            case State.TURN_RIGHT_1:
                self.angle_diff = angle_diff(self.current_angle, self.next_angle)
                print(f"angle diff: {self.angle_diff * 180 / math.pi}")
                self.turn()
            case State.TURN_LEFT_1:
                self.angle_diff = angle_diff(self.current_angle, self.next_angle)
                print(f"angle diff: {self.angle_diff * 180 / math.pi}")
                self.turn()
            case State.TURN_RIGHT_2:
                self.angle_diff = angle_diff(self.current_angle, self.next_angle)
                print(f"angle diff: {self.angle_diff * 180 / math.pi}")
                self.turn()
            case State.TURN_LEFT_2:
                self.angle_diff = angle_diff(self.current_angle, self.next_angle)
                print(f"angle diff: {self.angle_diff * 180 / math.pi}")
                self.turn()
            case State.MOVE_STRAIGHT_Y_LEFT:
                self.distance_diff = distance_diff(self.current_y, self.next_y)
                print(f"distance diff: {self.distance_diff}")
                self.move_straight()
            case State.MOVE_STRAIGHT_Y_RIGHT:
                self.distance_diff = distance_diff(self.current_y, self.next_y)
                print(f"distance diff: {self.distance_diff}")
                self.move_straight()
            case State.FOUND_OBJECT:
                self.find_object()
            case State.COMPLETE_SEARCH:
                self.complete_search()
            case default:
                raise Exception("Robot not in valid state.")


def main(args=None):
    rclpy.init(args=args)
    node = Agent()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
