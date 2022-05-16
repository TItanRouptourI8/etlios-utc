#!/usr/bin/python3

import rospy
import time
import math
import numpy
from std_msgs.msg import Int32,String,Float32MultiArray
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
class odometrieHandler:

    def __init__(self):
        self.posX=0
        self.posY=0
        self.angle=0
        self.lastime = 0
        self.brodcaster = tf.TransformBroadcaster()
        self.syncWithGyro()
        self.rpmsSubcriber=rospy.Subscriber("rpm_real",Point, self.computeOdom)
        self.odomPublisher = rospy.Publisher("odom", Odometry, queue_size = 50)



    def syncWithGyro(self):
        print("Syncing with Gyroscope, please wait and DONT MOVE THE ROBOT")
        raw_echant = 0
        for i in range(100):
            data : Imu = rospy.wait_for_message("/imu", Imu)
            quaternion = data.orientation
            explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            raw_echant += tf.transformations.euler_from_quaternion(explicit_quat)[2]
        print("Got 100 mesures")
        mean = raw_echant / 100
        self.angle = mean
        print("Sync was successful !, actual angle is : ", self.angle * 57, "°")




    def publishOdom(self, x,y,theta,vx,vy,w, sec, nsec):
        #handle tf :
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        o = Odometry()
        o.header.frame_id="odom"
        o.pose.pose.orientation.x = quaternion[0]
        o.pose.pose.orientation.y = quaternion[1]
        o.pose.pose.orientation.z = quaternion[2]
        o.pose.pose.orientation.w = quaternion[3]
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.position.z = 0.0
        o.twist.twist.linear.x = vx
        o.twist.twist.linear.y = vy
        o.twist.twist.angular.z = w
        self.odomPublisher.publish(o)


    def computeOdom(self, msg):
        if self.lastime ==0:
            a = rospy.Time.now()
            self.lastime = a.secs +a.nsecs*(10**(-9))
            return
        b = rospy.Time.now()
        dt = (b.secs + b.nsecs*(10**(-9))) - self.lastime
        rpmLeft = msg.x
        rpmRight = msg.y
        velLeft = (((2*math.pi)/60)*rpmLeft)*0.03
        velRight = (((2 * math.pi) / 60) * rpmRight) * 0.03

        vel_moy = (velLeft + velRight)/2
        vel_ang = (velRight - velLeft)/(0.168)

        deltaX = vel_moy*math.cos(self.angle)*dt
        deltaY = vel_moy*math.sin(self.angle)*dt
        deltaTh = vel_ang*dt

        self.posX = self.posX + deltaX
        self.posY = self.posY + deltaY
        self.angle = self.angle + deltaTh
        self.publishOdom(self.posX, self.posY, self.angle,vel_moy*math.cos(self.angle),vel_moy*math.sin(self.angle),vel_ang,b.secs, b.nsecs)
        self.lastime =  b.secs + b.nsecs*(10**(-9))


if __name__ == '__main__':
    rospy.init_node("odometrie")
    o = odometrieHandler()

    while not rospy.is_shutdown():
        pass


