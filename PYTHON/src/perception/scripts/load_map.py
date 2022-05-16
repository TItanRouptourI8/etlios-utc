#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapRequest, GetMapResponse

import pickle

rospy.init_node('load_map', anonymous=True)

filename = rospy.get_param("~filename", "map.txt")

file = open(filename, 'rb')

msg = OccupancyGrid()
msg = pickle.load(file)
file.close()

pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)

pub.publish(msg)

def cb(req):
    rep = GetMapResponse(map=msg)
    return rep

s = rospy.Service('static_map', GetMap, cb)
rospy.spin()