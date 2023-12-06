import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from odom_helper import TFHelper, euler_from_quaternion
import time 
from enum import Enum
import math 

NEOTO_LENGTH = 1

class State(Enum): 
    MOVE_STRAIGHT_X_RIGHT = "move straight in x direction"
    MOVE_STRAIGHT_X_LEFT = "move straight in x direction"
    MOVE_STRAIGHT_y = "move straight in y direction"
    TURN_lEFT_2 = "Turn 90 degrees left"
    TURN_RIGHT_2 = "Turn 90 degrees right"
    TURN_lEFT_1 = "Turn 90 degrees left"
    TURN_RIGHT_1 = "Turn 90 degrees right"
    FOUND_OBJECT = "Found object"
    COMPLETE_SERACH = "Completed area search"

class Lawnmower(Node):
    def __init__(self):
        super().__init__('lawnmower')
        self.state = "move_straight"
        self.scan = self.create_subscription(LaserScan, 'scan', self.parse_scans, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.scan_msg = None
        self.closest_angle = 0
        #Hard coded now, but brain will control later
        # in meters
        self.init_x = 0 
        self.init_y = 0
        self.current_x = 0
        self.current_y = 0
        self.next_y = 0
        self.max_x = 6
        self.max_y = 6
        self.linear_speed = 0.6 # CHANGE VALUE 
        self.angular_speed = 0.5
        self.init_angle = 0
        self.current_angle = 0
        ## orienting robot in direction of most weighted/closest objects 
        timer_period = 1
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.transform_helper = TFHelper(self)
        self.threshold = .1
        self.timer = self.create_timer(timer_period, self.run_loop)



    def move_straight_X(self):
        '''
        Moves robot straight
        '''
    
        self.move.linear.x = self.linear_speed
        self.publisher.publish(self.move)
    def turn_right(self):
        '''
        Move right 
        '''
        self.move.angular.x = self.angular_speed
        self.publisher.publish(self.move)

    def turn_left (self):
        '''
        Turn left
        '''
        self.move.angular.x = self.angular_speed
        self.publisher.publish(self.move)

    def check_object(self):
        '''
        Check if the object is in the current area 
        '''
        if self.x_object < self.threshold and self.y_object < self.threshold: 
            return True 
        return False
    def find_object(): 
        """
        Publish message to main brain 
        """
        print("Send coordiante to main brain, activating other robots to move to locaization")
    def complete_search():
        print("Send message that area assigned is searched")
    def choose_state(self): 
        # Final States
        if self.check_object(): 
            self.state = State.FOUND_OBJECT
        if self.next_y >= self.max_y:
            self.state = State.COMPLETE_SERACH
        # Search Loop
        if self.state == State.MOVE_STRAIGHT_X_RIGHT and self.current_x >= self.max_x: 
            self.state = State.TURN_lEFT
        elif self.state == State.TURN_lEFT_1 and self.current_angle >= 90: 
            self.state = State.MOVE_STRAIGHT_y
            self.next_y = self.current_y + NEOTO_LENGTH
        elif self.state == State.MOVE_STRAIGHT_y and self.current_y >= self.next_y: 
            self.state = State.TURN_lEFT_2
        elif self.state == State.TURN_lEFT_2 and self.current_angle >= 180:
            self.state = State.MOVE_STRAIGHT_X_LEFT
        elif self.state == State.MOVE_STRAIGHT_X_LEFT and self.current_x <= self.min_x: 
            self.state = State.TURN_RIGHT_1
        elif self.state == State.TURN_RIGHT_1 and self.current_angle >= 90: 
            self.state = State.MOVE_STRAIGHT_y
            self.next_y = self.current_y + NEOTO_LENGTH
        elif self.state == State.MOVE_STRAIGHT_y and self.current_y >= self.next_y: 
            self.state = State.TURN_RIGHT_2
        elif self.state == State.TURN_RIGHT_2 and self.current_angle >= 0:
            self.state = State.MOVE_STRAIGHT_X_RIGHT

    def run_loop(self):
        '''
        Switches state from predator to prey when the robot "tags" something/someone.
        '''


        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           time.localtime)
        self.current_x = new_pose.position.x
        self.current_y = new_pose.position.y
        _, _, yaw_z  = euler_from_quaternion(new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w)
        self.current_angle = yaw_z * 180 / math.pi
        self.check_object()
        match self.state:
            case State.MOVE_STRAIGHT_X_LEFT:
                self.move_straight_X()
            case State.MOVE_STRAIGHT_X_RIGHT:
                self.move_straight_X()
            case State.TURN_RIGHT_1:
                self.turn_right()
            case State.TURN_lEFT_1:
                self.turn_left()
            case State.TURN_RIGHT_2:
                self.turn_right()
            case State.TURN_lEFT_2:
                self.turn_left()
            case State.MOVE_STRAIGHT_y:
                self.move_straight_y()
            case State.FOUND_OBJECT: 
                self.find_object()
            case State.COMPLETE_SERACH: 
                self.complete_search()
            case default:
                raise Exception("Robot not in valid state.")

def main(args=None):
    rclpy.init(args=args)
    node = Lawnmower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()