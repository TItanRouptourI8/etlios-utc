#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import pickle

rospy.init_node('save_map', anonymous=True)

filename = rospy.get_param("filename", "map.txt")


def callback(msg):
    with open(filename, 'wb') as file:
        pickle.dump(msg, file)
        rospy.signal_shutdown("Finish")

    
rospy.Subscriber("/map", OccupancyGrid, callback)

rospy.spin()