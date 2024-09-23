import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PointStamped
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from time import sleep
import time
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.6

LIDAR_RIGHT_TURN_DISTANCE = 0.55
LIDAR_LEFT_TURN_DISTANCE = 0.55
LOCATION_PING = 5

SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90


class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False

        self.path = []
        self.last_recorded_time = time.time()
        self.find_wall = True
        self.bot_turning = False
        self.turned = False
        self.out_file = f"coords.txt"

        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.subscriber3 = self.create_subscription(
            PointStamped,
            '/TurtleBot3Burger/gps',
            self.listener_callback3,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_wall_dist = 0


    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)



    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
        self.pose_saved=position
           
        return None

    def listener_callback3(self, msg3):
        x = msg3.point.x
        y = msg3.point.y

        curr_time = time.time()
        if curr_time - self.last_recorded_time >= LOCATION_PING:
            self.path.append((x,y))
            self.last_recorded_time = curr_time
        return None

    def save_path_to_file(self):
        with open(self.out_file, 'w+') as f:
            for pos in self.path:
                f.write(f"{pos[0]}, {pos[1]}\n")

    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return

        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        if self.find_wall:
            self.go_straight()
            if front_lidar_min < LIDAR_AVOID_DISTANCE:
                self.last_wall_dist = right_lidar_min
                self.find_wall = False
        else:
            if front_lidar_min < LIDAR_AVOID_DISTANCE:
                #self.cmd.linear.x = 0.00
                self.bot_turning = False
                if right_lidar_min < LIDAR_RIGHT_TURN_DISTANCE and left_lidar_min < LIDAR_LEFT_TURN_DISTANCE:
                    self.bot_turning = True
                    self.cmd.linear.x = 0.25
                    self.cmd.angular.z = 5.0
                elif right_lidar_min < LIDAR_RIGHT_TURN_DISTANCE:
                    self.bot_turning = True
                    self.cmd.linear.x = 0.5
                    self.cmd.angular.z = 5.0
                    #turn left/CounterCW
                elif left_lidar_min < LIDAR_LEFT_TURN_DISTANCE:
                    self.bot_turning = True
                    self.cmd.linear.x = 0.5
                    self.cmd.angular.z = -5.0
                    #turn right
                else:
                    #turn left
                    self.bot_turning = True
                    self.cmd.linear.x = 0.5
                    self.cmd.angular.z = 5.0
            else:
                if right_lidar_min < LIDAR_RIGHT_TURN_DISTANCE and left_lidar_min < LIDAR_LEFT_TURN_DISTANCE:
                    self.go_straight()
                elif right_lidar_min < LIDAR_RIGHT_TURN_DISTANCE:
                    self.go_straight()
                elif left_lidar_min < LIDAR_LEFT_TURN_DISTANCE:
                    #turn right
                    self.bot_turning = True
                    self.cmd.linear.x = 0.5
                    self.cmd.angular.z = -5.0
                    #set bool to complete turn
                else:
                    #turn right
                    self.bot_turning = True
                    self.cmd.linear.x = 0.5
                    self.cmd.angular.z = -5.0

        self.publisher_.publish(self.cmd)    

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
           
           
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
    
    def go_straight(self):
        self.bot_turning = False
        self.turn = False
        self.cmd.linear.x = 0.5
        #self.cmd.angular.z = 0.0
        reading = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])

        #if self.last_wall_dist < reading + (reading * 0.1):
        #    self.cmd.angular.z = -0.001
        #elif self.last_wall_dist > reading - (reading * 0.1):
        #    self.cmd.angular.z = 0.001
        #else:
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.turtlebot_moving = True


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    try:
        rclpy.spin(random_walk_node)
    except KeyboardInterrupt:
        random_walk_node.save_path_to_file()
        random_walk_node.get_logger().info("End")
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
