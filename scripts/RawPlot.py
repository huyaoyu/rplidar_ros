#!/usr/bin/env python
# Lay some imports here to let the pdb work properly.

import os

#
# Description
# ===========
#
# A node is subscribed under the topic /scan which is published
# by the RPLIDAR A2 model. This node will count the number of 
# Infs in the data array of scan.ranges. A simple ROS_INFO is
# created to log the ratio of the Infs with respect to all the
# data stored in scan.ranges.
#
# Data
# ====
#
# Created: 2017-11-08
#
# Author
# ======
#
# Yaoyu Hu <huyaoyu@sjtu.edu.cn>
# 

# ========= ROS packages. ============

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# ========= Utility packages. ========

import matplotlib.pyplot as plt
import numpy as np

# ========= Generic python packages. ========

import math
import copy

# ======= File-wide global variables. ========

BUFFER_SIZE = 10                    # The size of the buffer.
buffer      = np.zeros(BUFFER_SIZE) # The buffer to hold the ratios.
posInBuff   = 0                     # Current index/position pointing into buffer.

# ======= Class definitions. =======

class RawPlotNode(object):
    """docstring for RawPlotNode"""
    def __init__(self):
        super(RawPlotNode, self).__init__()

        self.rePubScan = LaserScan()

        rospy.Subscriber('scan', LaserScan, self.callback)
        
    def callback(self, data):
        """
        Callback function of ROS node.

        Arguments:
        data - should be a sensor_msgs/LaserScan typed object.

        Return values:
        None.
        """

        # Global variables for future use.
        global DU, buffer, posInBuff

        # Create two ndarrays from data.
        degrees, ranges = transfer_from_scan_to_list(data)

        # Evaluate the data.
        nInfs, indicatorRanges = evaluate_data(degrees, ranges)

        # Overwrite current data in buffer.
        buffer[posInBuff] = 100.0 * nInfs / degrees.size

        # Log information.
        rospy.loginfo("%d data points received, with %d Infs (% 4.2f%%, % 4.2f%%).",
         degrees.size, nInfs, buffer[posInBuff], buffer.sum() / BUFFER_SIZE)

        # DU.on_running(degrees, ranges)

        # Shift position in buffer.
        posInBuff += 1

        if posInBuff == BUFFER_SIZE:
            posInBuff = 0

        # Copy data to rePubScan
        self.rePubScan = copy.deepcopy(data)
        self.rePubScan.ranges = indicatorRanges

# ============= Function definitions. ===============

def RAD2DEG(rad):
    return rad / math.pi * 180.0

def transfer_from_scan_to_list(scan):
    """
    Transfer the data of sensor_msgs/LaserScan into two separate lists.
    There will be two newly generated numpy ndarrays served as the return values.

    Arguments:
    scan - the original sensor_msgs/LaserScan data

    Return values:
    degrees - numpy ndarray, the degrees of data points, deg
    ranges  - numpy ndarray, the distance of measured points, m
    """

    # Estimate the number of data points.
    count = int(scan.scan_time / scan.time_increment)
    
    # Pre-allocation for the actual data.
    degrees = np.zeros(count)

    for i in range(count):
        degree = RAD2DEG(scan.angle_min + scan.angle_increment * i);

        degrees[i] = degree
        # ranges[i]  = scan.ranges[i]
        # rospy.loginfo(": [%f, %f]", degree, scan.ranges[i]);

    # Create a ndarray directly from a list.
    ranges = np.array(scan.ranges)

    return degrees, ranges

def evaluate_data(degrees, ranges):
    """
    Find out the total number of Infs in ranges. Argument degrees is not used.

    Arguments:
    degrees - ndarray, the return value of transfer_from_scan_to_list()
    ranges  - ndarray, the return value of transfer_from_scan_to_list().

    Return values:
    count - the total number of Infs in ranges.
    indictorRanges - list, the indicators for plotting in RViz.
    """

    count = 0

    indicatorRanges = [1.0] * ranges.size

    for i in range(ranges.size):
        r = ranges[i]

        if math.isinf(r):
            count += 1
            indicatorRanges[i] = 0.0

    return count, indicatorRanges

if __name__ == '__main__':
    rospy.init_node('RawPlot', anonymous=True)

    rpn = RawPlotNode()

    rate = rospy.Rate(10)

    pub = rospy.Publisher('ReScan', LaserScan, queue_size=10)

    while not rospy.is_shutdown():
        pub.publish(rpn.rePubScan)

        rate.sleep()
