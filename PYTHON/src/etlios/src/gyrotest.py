#!/usr/bin/python3
import rospy
import tf
from sensor_msgs.msg import Imu
rospy.init_node("imutest")

def printEuleur(imu : Imu):
    quaternion = (imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    print(yaw * 57.29)

sub = rospy.Subscriber("/imu", Imu,printEuleur )

while not rospy.is_shutdown():
    pass