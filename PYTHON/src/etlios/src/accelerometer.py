#!/usr/bin/python3
from mpu6050 import mpu6050
import rospy
import tf
from sensor_msgs.msg import Imu
sensor = mpu6050(0x68)
import math
linear_acceleration_stdev_ =(400 / 1000000.0) * 9.807
angular_velocity_stdev_ =  0.05 * (math.pi / 180.0)
pitch_roll_stdev_ = 1.0 * (math.pi / 180.0)
yaw_stdev_ = 5.0 * (math.pi / 180.0)
angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_
linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_
pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_
yaw_covariance = yaw_stdev_ * yaw_stdev_
if __name__ == '__main__':
    rospy.init_node("odometrie")

    rate = rospy.Rate(20)
    print("coucou")

    while not rospy.is_shutdown():
        accelerometer_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()
        msg = Imu()
        msg.header.frame_id="imu"
        msg.header.stamp = rospy.Time.now()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, gyro_data.z)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
        msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
        msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;

        msg.angular_velocity_covariance[0] = angular_velocity_covariance;
        msg.angular_velocity_covariance[4] = angular_velocity_covariance;
        msg.angular_velocity_covariance[8] = angular_velocity_covariance;

        msg.orientation_covariance[0] = pitch_roll_covariance;
        msg.orientation_covariance[4] = pitch_roll_covariance;
        msg.orientation_covariance[8] = yaw_covariance;


        rate.sleep()
