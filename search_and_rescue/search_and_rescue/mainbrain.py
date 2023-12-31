import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32MultiArray
from geometry_msgs.msg import Pose
from . import odom_helper as od
import numpy as np
from .load_map import LoadMap
from .constants import Names


class Brain(Node):
    """
    Ros2 Node that instantiates a "centralized" brain who communicates to different robots. Capabilities include:
        - Using transforms to track each robot's location in the map frame
        - Splitting a given map's dimensions for a particular number of robots
        - publishing corresponding information to each robot about its map bounds 

    """

    def __init__(self):
        super().__init__("main_brain")
        # loading map node to obtain information about map
        self.map = LoadMap(self)
        # the initial pose of each neato in the map frame
        self.map_1_init_pose = []
        self.map_2_init_pose = []
        self.map_3_init_pose = []
        # the position of each neato in terms of the map frame
        self.map_1_pose = []
        self.map_2_pose = []
        self.map_3_pose = []

        # making a corresponding subscription to odom for every neato we have
        # for name in Names:
        #     self.subscriber = self.create_subscription(
        #         Odometry, f"{name.value}/odom", self.ruhroh(name), 10
        #     )
        #     self.publisher = self.create_publisher(
        #         UInt32MultiArray, f"{name.value}/map_path", 10
        #     )

        # hard-coded subscribers/publishers for two robots 
        self.ally_subscriber = self.create_subscription(
            Odometry, "ally/odom", self.ruhroh(Names.ALLY), 10
        )
        self.ally_publisher = self.create_publisher(
            UInt32MultiArray, "ally/map_path", 10
        )
        self.billy_subscriber = self.create_subscription(
            Odometry, "billy/odom", self.ruhroh(Names.BILLY), 10
        )
        self.billy_publisher = self.create_publisher(
            UInt32MultiArray, "billy/map_path", 10
        )

        # timer to run the node functionality
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.test_run)

    def ruhroh(self, name):
        """
        "Callback function" that returns the correct callback function to be run, given that there are multiple
        robots and each has a different initial starting point in the map.

        To make this project scalable, it is important that we find a way to initialize as many subscribers and publishers
        as we have robots for a given run. In the init, we are using a for loop to create new subscribers for the brain to
        receive each neato's odom pose. However, this one line of code only takes in one callback function. Therefore this
        "dummy" callback function is created such that it will refer the for loop to a new callback function for every number
        of neatos we are told there are.

        Args:
            i: An integer representing the number loop or number neato that we are subcribing to.
        Returns:
            A function representing the correct neato transform callback function for the subscriber.
        """
        match name:
            case Names.ALLY:
                return self.neato1_transform
            case Names.BILLY:
                return self.neato2_transform
            case default:
                raise Exception(
                    "name of robot is not recognized."
                )

    def split_map(self, numneato):
        """
        Given a value of how many neatos are "searching", split the map evenly amongst them then publish the boundaries to each corresponding neato.

        Args:
            numneato: An integer representing the number of neatos that will be searching the map.

        """
        # given in number of pixels, or number of "0s" in the matrix
        width_pixels = self.map.map_width
        height_pixels = self.map.map_height

        # dimensions in meters
        # the space between pixels is the num meters irl on map (determined by resolution)
        map_width_meters = (width_pixels - 1) * self.map.map_resolution
        map_height_meters = (height_pixels - 1) * self.map.map_resolution

        # how much distance given to each neato
        width_per_neato = int(map_width_meters / numneato)
        height_per_neato = int(map_height_meters / numneato)

        # publishing to each neato its bounds
        map_bounds_neato1 = UInt32MultiArray()
        map_bounds_neato1.data = [
            width_per_neato,
            height_per_neato,
        ]

        map_bounds_neato2 = UInt32MultiArray()
        map_bounds_neato2.data = [
            width_per_neato,
            height_per_neato,
        ]
        # publish the map bounds to each robot
        self.ally_publisher.publish(map_bounds_neato1)
        self.billy_publisher.publish(map_bounds_neato2)

    def neato1_transform(self, msg: Odometry):
        """
        calculating the transforms between given robot's odom position to the global frame
        essentially finding where the robot is in the map.

        Args:
            msg:Odometry: a msg type Odometry that represents the odom pose of the neato, given by a publisher.
        """
        new_1_odom_pose = od.convert_pose_to_xy_and_theta(
            msg.pose.pose
        )  # grabbing message from subscription from class pose of attribute pose

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
        rel_pose = (
            t1_matrix @ t2_matrix
        )  # stores the position of where robot is in the map frame

        self.map_1_pose = rel_pose[:, 2]  # saved as a 3 integer array

        return

    def neato2_transform(self, msg: Odometry):
        """
        calculating the transforms between given robot's odom position to the global frame
        essentially finding where the robot is in the map
        """
        new_2_odom_pose = od.convert_pose_to_xy_and_theta(
            msg.pose.pose
        )  # grabbing message from subscription from class pose of attribute pose

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
        rel_pose = (
            t1_matrix @ t2_matrix
        )  # stores the position of where robot is in the map frame

        self.map_2_pose = rel_pose[:, 2]  # saved as a 3 integer array

        return

    def neato3_transform(self, msg: Odometry):
        """
        calculating the transforms between given robot's odom position to the global frame
        essentially finding where the robot is in the map
        """
        new_3_odom_pose = od.convert_pose_to_xy_and_theta(
            msg.pose.pose
        )  # grabbing message from subscription from class pose of attribute pose

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
        rel_pose = (
            t1_matrix @ t2_matrix
        )  # stores the position of where robot is in the map frame

        self.map_3_pose = rel_pose[:, 2]  # saved as a 3 integer array

        return

    def tell_robot_it_found_object(self):
        pass

    def test_run(self):
        """
        A looped function that is used to test/run the functions.
        """
        print("hi!")
        self.split_map(2) # 2 robots are instantiated to split the map 


def main(args=None):
    rclpy.init(args=args)

    n = Brain()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
