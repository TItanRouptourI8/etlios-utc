#!/usr/bin/python3.7
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage


def getFrame():
    success, frame = camera.read()
    ret, buffer = cv2.imencode('.jpg', frame)

    return buffer

if __name__ == '__main__':
    rospy.init_node("camera")
    rate = rospy.Rate(50)
    pb = rospy.Publisher('/camera/image/compressed',CompressedImage)
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while not rospy.is_shutdown():
        buff = getFrame()
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(buff).tostring()
        pb.publish(msg)

        rate.sleep()
