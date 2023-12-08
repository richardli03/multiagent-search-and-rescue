import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import MultiArrayDimension, UInt8MultiArray
from geometry_msgs.msg import Pose
import odom_helper as od
import numpy as np
from search_and_rescue/load_map import LoadMap

numneato = 5 # will be in the launch file, specifying how many neatos there are 

class Brain(Node):
    """
    
    """
    def __init__(self):
        super().__init__('main brain')
        self.map_1_init_pose = [] # the initial starting point of each neato 
        self.map_2_init_pose = []
        self.map_3_init_pose = []
        self.map_1_pose = [] # the position of each neato in terms of the map frame 
        self.map_2_pose = []
        self.map_3_pose = []

        # making subscriptions for every neato we have 
        for i in range():
            self.subscriber = self.create_subscription(Odometry, f"neato{i}/odom", self.ruhroh(i),10)
            self.publisher = self.create_publisher(UInt8MultiArray, f"neato{i}/map_path", 10)


    def split_map(self,numneato):
        """
        Given a value of how many neatos are "searching", split the map evenly amongst them then publish the boundaries to each corresponding neato.
            Args:
        numneato: An integer representing the number of neatos that will be searching the map.

        """
        rclpy.init()
        n = LoadMap() # anmol's map generating node 

        # given in number of pixels, or number of "0s" in the matrix 
        width_pixels = n.get_map().width
        height_pixels = n.get_amp().height

        # dimensions in meters 
        # the space between pixels is the num meters irl on map (determined by resolution) 
        map_width = (width_pixels - 1) * n.resolution 
        map_height = (height_pixels - 1) * n.resolution

        # how much distance given to each neato 
        width_per_neato = map_width / numneato
        height_per_neato = map_height / numneato

        # publishing to each neato its obunds 
        map_bounds_neato1 = UInt8MultiArray()

        map_bounds_neato1.layout.dim.append(MultiArrayDimension())
        map_bounds_neato1.layout.dim[0].size = 8 # how many elements we input 
        map_bounds_neato1.layout.dim[0].stride = 2 # the number of integers skipped to go to next line of array 
        map_bounds_neato1.data = [0, 0, width_per_neato, 0, 0, height_per_neato, height_per_neato, height_per_neato] # order: bot left corner, bot right corner, top left corner, top right corner

        rclpy.loginfo("Publishing bounds data: {}".format(map_bounds_neato1.data))
        self.publisher.publish(map_bounds_neato1)
        # giving each neato its bounds in terms of its own odom frame (aka, since mvp all starts the neatos where we want them, i give them all the same coord bounding boxes)


    
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


    def main(args=None):

        rclpy.spin(n)
        rclpy.shutdown()


    if __name__ == "__main__":
        main()
       
self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

 f"{self.get_name()}/odom"


