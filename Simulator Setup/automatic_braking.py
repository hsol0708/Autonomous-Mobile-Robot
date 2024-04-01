#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import math

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class AutomaticBraking(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('automatic_braking')
        """
        One publisher should publish to the /drive topic with a AckermannDrive drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        odom_topic = '/odom'
        drive_topic = '/drive'

        self.speed = 0.0

        """
        The /scan lidar subscriber is provided due to the lidar topic being configured as "Best Effort" where the messages are sent as quickly as possible
        without regard for if the receiver is properly handling them. This is different from the default "Reliable" quality of service configuration which 
        will resend messages until they are received. For the rest of your subscribers and publishers you SHOULD NOT use the qos_profile generated below.
        """
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            qos_profile = qos_profile
            )
        
        # TODO: create ROS subscribers and publishers for odom and drive
        self.odom_subscriber = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
            )
        
        self.publisher_ = self.create_publisher(AckermannDrive, drive_topic, 10)
 
        self.forward_velocity = 0


    def odom_callback(self, odom_msg):
        # TODO: update current speed

        self.forward_velocity = odom_msg.twist.twist.linear.x

        

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        angle_array = np.arange(start=scan_msg.angle_min, stop=scan_msg.angle_max, step=scan_msg.angle_increment)
        #print angle min and angle max
        # print(scan_msg.angle_min, scan_msg.angle_max)

        speed_array = (np.cos(angle_array) * self.forward_velocity)
        # print(speed_array)

        denominator = np.maximum(-speed_array, 0)
        # print(denominator)
        ittc = scan_msg.ranges / denominator
        

        
        # print()
        mask = np.isfinite(ittc) & (scan_msg.ranges != 0) & (angle_array > - np.pi/2) & (angle_array < np.pi/2)
        print(mask)
        
        # for i in range(len(ittc)):
        #     print(ittc[i], speed_array[i], scan_msg.ranges[i])

        ittc = ittc[mask]
        # if len(ittc) > 0:
        #     print(min(ittc), max(ittc))
        

        # TODO: publish command to brake
        for time in ittc:
            
            if (time > 0.01) & (time < 0.25):
                
                msg = AckermannDrive()
                msg.speed = 0.0
                for i in range(200):
                    print("stopping")
                    self.publisher_.publish(msg)
                break

        

def main(args=None):
    rclpy.init(args=args)
    automatic_braking = AutomaticBraking()
    rclpy.spin(automatic_braking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    automatic_braking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
