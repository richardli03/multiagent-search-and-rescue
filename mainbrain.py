import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import odom_helper as od
import numpy as np

numneato = 5

class Brain(Node):
    def __init__(self):
        super().__init__('main brain')
        self.map_1_init_pose = [] # the initial starting point of each neato 
        self.map_2_init_pose = []
        self.map_3_init_pose = []
        self.map_1_pose = [] # the position of each neato in terms of the map frame 
        self.map_2_pose = []
        self.map_3_pose = []

        for i in range():
            self_subscriber = self.create_subscription(Odometry, f"neato{i}/odom", self.ruhroh(i),10)

    
    def ruhroh(self, i):
       '''
       Returns a callback corresponding to which robot it is 
       '''
       match i:
        case 1:
            return self.neato1_transform
        case 2: 
            return self.neato2_transform
        case 3:
            return self.neato3_transform
        case default:
            return self.neato1_transform
          

    def neato1_transform(self, msg: Odometry):
        '''
        calculating the transforms between given robot's odom position to the global frame
        essentially finding where the robot is in the map 
        '''
        new_1_odom_pose = od.convert_pose_to_xy_and_theta(msg.pose.pose) # grabbing message from subscription from class pose of attribute pose 
       

        # define parts of map->odom and robot->odom homogeneous matrix
        t1_x = self.map_1_init_pose[0]
        t1_y = self.map_1_pose[1]
        t1_theta = self.map_1_init_pose[2]
        t2_x = new_1_odom_pose[0]
        t2_y = new_1_odom_pose[1]
        t2_theta = new_1_odom_pose[2]

        # define transformation matrices and finding relative pose using matrix multiplication (see report for how this math works)
        t1_matrix = np.linalg.inv(
            np.array(
                [
                    [np.cos(t1_theta), -np.sin(t1_theta), t1_x],
                    [np.sin(t1_theta), np.cos(t1_theta), t1_y],
                    [0, 0, 1],
                ]
            )
        )
        t2_matrix = np.array(
            [
                [np.cos(t2_theta), -np.sin(t2_theta), t2_x],
                [np.sin(t2_theta), np.cos(t2_theta), t2_y],
                [0, 0, 1],
            ]
        )
        rel_pose = t1_matrix @ t2_matrix # stores the position of where robot is in the map frame 

        self.map_1_pose = rel_pose[:,2] # saved as a 3 integer array 

        return

    def neato2_transform(self, msg: Odometry):
        '''
        calculating the transforms between given robot's odom position to the global frame
        essentially finding where the robot is in the map 
        '''
        new_2_odom_pose = od.convert_pose_to_xy_and_theta(msg.pose.pose) # grabbing message from subscription from class pose of attribute pose 
    

        # define parts of map->odom and robot->odom homogeneous matrix
        t1_x = self.map_2_init_pose[0]
        t1_y = self.map_2_init_pose[1]
        t1_theta = self.map_2_init_pose[2]
        t2_x = new_2_odom_pose[0]
        t2_y = new_2_odom_pose[1]
        t2_theta = new_2_odom_pose[2]

        # define transformation matrices and finding relative pose using matrix multiplication (see report for how this math works)
        t1_matrix = np.linalg.inv(
            np.array(
                [
                    [np.cos(t1_theta), -np.sin(t1_theta), t1_x],
                    [np.sin(t1_theta), np.cos(t1_theta), t1_y],
                    [0, 0, 1],
                ]
            )
        )
        t2_matrix = np.array(
            [
                [np.cos(t2_theta), -np.sin(t2_theta), t2_x],
                [np.sin(t2_theta), np.cos(t2_theta), t2_y],
                [0, 0, 1],
            ]
        )
        rel_pose = t1_matrix @ t2_matrix # stores the position of where robot is in the map frame 

        self.map_2_pose = rel_pose[:,2] # saved as a 3 integer array 
        
        return 

    def neato3_transform(self, msg: Odometry):
        '''
        calculating the transforms between given robot's odom position to the global frame
        essentially finding where the robot is in the map 
        '''
        new_3_odom_pose = od.convert_pose_to_xy_and_theta(msg.pose.pose) # grabbing message from subscription from class pose of attribute pose 


        # define parts of map->odom and robot->odom homogeneous matrix
        t1_x = self.map_3_init_pose[0]
        t1_y = self.map_3_init_pose[1]
        t1_theta = self.map_3_init_pose[2]
        t2_x = new_3_odom_pose[0]
        t2_y = new_3_odom_pose[1]
        t2_theta = new_3_odom_pose[2]

        # define transformation matrices and finding relative pose using matrix multiplication (see report for how this math works)
        t1_matrix = np.linalg.inv(
            np.array(
                [
                    [np.cos(t1_theta), -np.sin(t1_theta), t1_x],
                    [np.sin(t1_theta), np.cos(t1_theta), t1_y],
                    [0, 0, 1],
                ]
            )
        )
        t2_matrix = np.array(
            [
                [np.cos(t2_theta), -np.sin(t2_theta), t2_x],
                [np.sin(t2_theta), np.cos(t2_theta), t2_y],
                [0, 0, 1],
            ]
        )
        rel_pose = t1_matrix @ t2_matrix # stores the position of where robot is in the map frame 

        self.map_3_pose = rel_pose[:,2] # saved as a 3 integer array 
        
        return 

       
self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

 f"{self.get_name()}/odom"


