import rclpy

from rclpy.node import Node
import math
 

import numpy as np

from sensor_msgs.msg import LaserScan

from ackermann_msgs.msg import AckermannDrive

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

 

class GapFollowing(Node):

    """

    Implement Follow the Gap on the car

    This is just a template, you are free to implement your own node!

    """

 

    def __init__(self):

        super().__init__('gap_following')

 

        # Topics & Subs, Pubs

        lidarscan_topic = '/scan'

        drive_topic = '/drive'

 

        # Create ROS subscribers and publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, qos_profile = qos_profile)

        self.drive_pub = self.create_publisher(AckermannDrive, drive_topic, 10)

 

        # Constants for gap following
        self.window_size = 10  # Window size for averaging LIDAR points

        self.threshold_distance = 5.0  # Threshold distance to reject LIDAR points

        self.safety_bubble_size = 300  # Number of nearby points to set to 0 around the closest point

        self.gap_threshold = 0.7  # Threshold size of a gap to follow it

 

    def preprocess_lidar(self, ranges):

        """ Preprocess the LiDAR scan array by averaging over a window and rejecting high values """

 

        # Average over a window
        proc_ranges = np.convolve(ranges, np.ones(self.window_size) / self.window_size, mode='same')

 

        proc_ranges[proc_ranges > self.threshold_distance] = self.threshold_distance

 

        return proc_ranges

 

    def find_closest_point(self, ranges):

        """ Find the index of the closest LIDAR point """
        temp = ranges
        #temp[ranges <= 0.1] = self.threshold_distance
        
        return np.argmin(temp)

 

    def draw_safety_bubble(self, closest_idx, ranges):

        """ Draw a safety bubble around the closest point by setting nearby points to 0 """

        start_idx = max(0, closest_idx - self.safety_bubble_size)

        end_idx = min(len(ranges), closest_idx + self.safety_bubble_size)

        ranges[start_idx:end_idx] = 0.0

        return ranges

 

    def find_max_gap(self, free_space_ranges):

        """ Return the start index & end index of the largest gap in free_space_ranges """

        # Find consecutive non-zero elements that are greater than the gap threshold

 

        non_zero_segments = np.split(free_space_ranges, np.where(free_space_ranges < self.gap_threshold)[0])

        # print(non_zero_segments)

        max_gap = max(non_zero_segments, key=len, default=[])

        start_idx = free_space_ranges.tolist().index(max_gap[0])

        end_idx = start_idx + len(max_gap)

        return start_idx, end_idx

 

    def find_best_point(self, start_i, end_i, ranges):

        """ Find the index of the best goal point in the chosen gap """

        # Naive: Choose the furthest point within the gap

        #best_i = np.argmax(ranges[start_i:end_i])
        best_i = int((start_i + end_i)/2) - start_i # midpoint instead of furthest
        #print(f"Best_i: {best_i} has a range of {ranges[best_i + start_i]}")

 

        return best_i + start_i

 

    def lidar_callback(self, data):

        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDrive Message """

        angle_array = np.arange(start=data.angle_min, stop=data.angle_max, step=data.angle_increment)
        ranges = np.array(data.ranges)
        #print("scan")
        #for i in range(len(ranges)):
        #    print(float('%.3g' % angle_array[i]), ranges[i])
        mask = np.isfinite(ranges) & (angle_array > -np.pi*1/2) & (angle_array < np.pi*1/2)
        # for i in range(len(mask)):
        #     print(mask[i])
#        print(len(angle_array), len(ranges), len(mask))
        
        for i in range(len(ranges)): # get rid of zeros in lidar
            if i == 0:
                continue
            elif ranges[i] == 0:
                ranges[i] = ranges[i-1]
        ranges = ranges[mask] # Only consider the front 180 degrees
        angle_array = angle_array[mask]

        proc_ranges = self.preprocess_lidar(ranges)

 

        # Find closest point to LiDAR

        closest_idx = self.find_closest_point(proc_ranges)
 

        # Eliminate all points inside 'bubble' (set them to zero)

        free_space_ranges = self.draw_safety_bubble(closest_idx, proc_ranges)

 

        # Find max length gap

        start_i, end_i = self.find_max_gap(free_space_ranges)

        # print(f"Start: {start_i}, End: {end_i}")

 

        # Find the best point in the gap

        best_point_idx = self.find_best_point(start_i, end_i, ranges)
        # print(f"Safety bubble: {angle_array[closest_idx]*180/np.pi}, Best angle: {angle_array[best_point_idx]*180/np.pi} has a range of {ranges[best_point_idx]}")

        # print(f"Best Point: {best_point_idx}")

 

        # Publish Drive message

        drive_msg = AckermannDrive()

        drive_msg.speed = 1.0 if ranges[closest_idx] > .4 else 1.00 * math.exp(-0.1*(self.threshold_distance - ranges[closest_idx]) ) # Set your desired speed here

        steer_angle = -angle_array[int((best_point_idx - len(ranges) / 2.0))] * .5
        if steer_angle > math.pi/4:
            steer_angle = math.pi/4
        if steer_angle < -math.pi/4:
            steer_angle = -math.pi/4
        drive_msg.steering_angle = steer_angle

        self.drive_pub.publish(drive_msg)

 

 

def main(args=None):

    rclpy.init(args=args)

    gap_following = GapFollowing()

    rclpy.spin(gap_following)

 

    gap_following.destroy_node()

    rclpy.shutdown()

 

 

if __name__ == '__main__':

    main()
