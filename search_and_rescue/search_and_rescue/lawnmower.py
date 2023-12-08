import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from search_and_rescue.odom_helper import euler_from_quaternion, angle_diff, distance_diff
from enum import Enum
import math 
import time
from nav_msgs.msg import Odometry

NEOTO_LENGTH = .5

class State(Enum): 
    MOVE_STRAIGHT_X_RIGHT = "move straight in x direction right"
    MOVE_STRAIGHT_X_LEFT = "move straight in x direction left"
    MOVE_STRAIGHT_Y_LEFT = "move straight in y direction wiht next turn left"
    MOVE_STRAIGHT_Y_RIGHT = "move straight in y direction wiht next turn right"
    TURN_lEFT_2 = "Turn 90 degrees left 2"
    TURN_RIGHT_2 = "Turn 90 degrees right 2"
    TURN_lEFT_1 = "Turn 90 degrees left 1"
    TURN_RIGHT_1 = "Turn 90 degrees right 1"
    FOUND_OBJECT = "Found object"
    COMPLETE_SERACH = "Completed area search"

class Lawnmower(Node):
    def __init__(self):
        super().__init__('lawnmower')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.process_odom, 10)
        self.odom = None
        self.state = State.MOVE_STRAIGHT_X_RIGHT
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move = Twist()
        self.scan_msg = None
        #Hard coded now, but brain will control later
        # in meters
        self.init_x = 0 
        self.init_y = 0
        self.current_x = 0
        self.current_y = 0
        self.next_y = 0
        self.max_x = 2
        self.max_y = 5
        self.linear_speed = 0.3 # CHANGE VALUE 
        self.angular_speed = 0.1
        self.current_angle = 0
        self.next_angle = 0
        ## orienting robot in direction of most weighted/closest objects 
        timer_period = 1
        self.threshold = .1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.object_x = 3
        self.object_y = 20
        self.distance_diff = self.max_x
        self.angle_diff = math.pi / 2 


    def process_odom(self, odom):
        self.odom = odom 

    def move_straight(self):
        '''
        Moves robot straight
        '''
        self.move.angular.z = 0.0
        self.move.linear.x = self.linear_speed * self.distance_diff
        self.publisher.publish(self.move)
    def turn(self):
        '''
        turn based on angle diff 
        '''
        self.move.angular.z = self.angular_speed * self.angle_diff
        self.move.linear.x = 0.0 
        self.publisher.publish(self.move)

    def check_object(self):
        '''
        Check if the object is in the current area 
        '''
        if self.object_x == self.current_x and self.object_y == self.current_y: 
            return True 
        return False
    def find_object(self): 
        """
        Publish message to main brain 
        """
        self.move.angular.z = 0.0
        self.move.linear.x = 0.0 
        self.publisher.publish(self.move)
        print("Send coordiante to main brain, activating other robots to move to locaization")
    def complete_search(self):
        self.move.angular.z = 0.0
        self.move.linear.x = 0.0 
        self.publisher.publish(self.move)
        print("Send message that area assigned is searched")
    def choose_state(self): 

        # Final States
        if self.check_object(): 
            self.state = State.FOUND_OBJECT
        if self.next_y >= self.max_y:
            self.state = State.COMPLETE_SERACH
        # Search Loop
        if self.state == State.MOVE_STRAIGHT_X_RIGHT and self.distance_diff < self.threshold:
            print(self.distance_diff)
            print("SWITCH TO TURN LEFT 1")
            self.state = State.TURN_lEFT_1
            self.next_angle = self.current_angle - 90
            print(f"Next angle: {self.next_angle * 180 / math.pi}")
        elif self.state == State.TURN_lEFT_1 and self.angle_diff < 0 : 
            print("SWITCH TO TURN MOVE STRAIGHT Y")        
        elif self.state == State.MOVE_STRAIGHT_Y_LEFT and self.distance_diff < self.threshold: 
            self.state = State.MOVE_STRAIGHT_Y_LEFT
            self.next_y = self.current_y + NEOTO_LENGTH
            print("SWITCH TURN LEFT 2")
            self.state = State.TURN_lEFT_2
            self.next_angle = self.current_angle + math.pi/2 
            print(f"Next angle: {self.next_angle * 180 / math.pi}")
        elif self.state == State.TURN_lEFT_2 and self.angle_diff < self.threshold:
            print("SWITCH TURN MOVE STRAIGHT X LEFT ")
            print(f"Current Angle: {self.current_angle} --SWITCH")
            self.state = State.MOVE_STRAIGHT_X_LEFT
        elif self.state == State.MOVE_STRAIGHT_X_LEFT and self.distance_diff < self.threshold: 
            self.state = State.TURN_RIGHT_1
            self.next_angle = self.current_angle + math.pi/2 
            print(f"Next angle: {self.next_angle * 180 / math.pi}")
        elif self.state == State.TURN_RIGHT_1 and self.angle_diff < self.threshold: 
            self.state = State.MOVE_STRAIGHT_Y_RIGHT
            self.next_y = self.current_y + NEOTO_LENGTH
        elif self.state == State.MOVE_STRAIGHT_Y_RIGHT and self.distance_diff < self.threshold: 
            self.state = State.TURN_RIGHT_2
            self.next_angle = self.current_angle + math.pi/2 
            print(f"Next angle: {self.next_angle * 180 / math.pi}")
        elif self.state == State.TURN_RIGHT_2 and self.angle_diff < self.threshold:
            self.state = State.MOVE_STRAIGHT_X_RIGHT

    def run_loop(self):
        '''
        Switches state from predator to prey when the robot "tags" something/someone.
        '''
        new_pose : Odometry= self.odom
    
        if new_pose is None: 
            return
        self.current_x = new_pose.pose.pose.position.x
        self.current_y = new_pose.pose.pose.position.y
        print(f"pos: {self.current_x}, {self.current_y}\n")
        _, _, yaw_z  = euler_from_quaternion(new_pose.pose.pose.orientation.x,new_pose.pose.pose.orientation.y, new_pose.pose.pose.orientation.z, new_pose.pose.pose.orientation.w)
        self.current_angle = yaw_z 
        print(f"angle: {self.current_angle * 180 / math.pi}")
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
            case State.TURN_lEFT_1:
                self.angle_diff = angle_diff(self.current_angle, self.next_angle)
                print(f"angle diff: {self.angle_diff * 180 / math.pi}")
                self.turn()
            case State.TURN_RIGHT_2:
                self.angle_diff = angle_diff(self.current_angle, self.next_angle)
                print(f"angle diff: {self.angle_diff * 180 / math.pi}")
                self.turn()
            case State.TURN_lEFT_2:
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