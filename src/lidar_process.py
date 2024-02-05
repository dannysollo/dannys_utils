#!/usr/bin/env python3.6

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time


# Subscribe to Lidar data from rplidar. 
# Prune certain angles - currently include all as this requires testing.
# Determine the closest distance to the robot and the angle at which it occurs.
# If the closest distance is less than a threshold, send a fault on robotstatus topic.

def callback(data):
    # Range of angles to ignore
    mindist = 0.5
    ignored_angles = [[0, 0], [0, 0]]
    ranges = np.array(data.ranges)
    # Prune the ranges
    for i in range(len(ignored_angles)):
        ranges[ignored_angles[i][0]:ignored_angles[i][1]] = 100
    # Find the minimum distance and the angle at which it occurs
    min_distance = np.min(ranges)
    min_angle = np.argmin(ranges)
    if (min_distance < mindist):
        print("FAULT-PROXIMITY")
        pub = rospy.Publisher('fault', String, queue_size=10)
        pub.publish("FAULT-PROXIMITY")
    else:
        pub = rospy.Publisher('fault', String, queue_size=10)
        pub.publish("None")


def lidar_node():
    rospy.init_node('lidar_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()



if __name__ == '__main__':
    lidar_node()
