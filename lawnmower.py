import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from odom_helper import TFHelper
import time 

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
        self.max_x = 6
        self.max_y = 6
        self.linear_speed = 0.6 # CHANGE VALUE 
        ## orienting robot in direction of most weighted/closest objects 
        timer_period = 1
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.transform_helper = TFHelper(self)
        self.threshold = .1
        self.timer = self.create_timer(timer_period, self.run_loop)



    def move_straight(self, x, y):
        '''
        Moves robot straight until certain location (x,y)
        '''
        x_difference =  abs(x-self.current_x)
        y_difference =  abs(y - self.current_y)
        if x_difference > self.threshold and y_difference > self.threshold:
            self.move.linear.x = self.linear_speed
            self.publisher.publish(self.move)
            self.state = "move_straight"
        else: 
            self.state = "turn"
    def turn(self): 
        '''
        Switch state to turn left or right 
        '''
        side = abs(self.max_x -self.current_x)
        if side < self.threshold: 
            #Close to max x need to turn left 
            self.state = "turn_left"
        else: 
            #Close to min x, need to turn right 
            self.state = "turn_right"

    def move_up_right(self):
        '''
        Moves robot according to "predator" state behavior-- it moves towards the closest object.
        '''
        print(self.state)
    
        self.publisher.publish(self.move)
    def move_up_left(self):
        self.publisher.publish(self.move)
    def check_object(self):
        '''
        Check if the object is in the current area 
        '''
        if self.x_object < self.threshold and self.y_object < self.threshold: 
            self.state = "find_object"
    def find_object(): 
        """
        Publish message to main brain 
        """
        print("Send coordiante to main brain, activating other robots to move to locaization")
    def complete_search():
        print("Send message that area assigned is searched")


    def run_loop(self):
        '''
        Switches state from predator to prey when the robot "tags" something/someone.
        '''
        #logic for turning

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           time.localtime)
        self.current_x = new_pose.position.x
        self.current_y = new_pose.position.y
        self.check_object()
        match self.state:
            case "move_straight":
                self.move_straight()
            case "turn":
                self.turn()
            case "turn_right":
                self.move_up_right()
            case "turn_left":
                self.move_up_left
            case "find_object": 
                self.find_object
            case "complete_area": 
                self.complete_search
            case default:
                return "stuck"

def main(args=None):
    rclpy.init(args=args)
    node = Lawnmower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()