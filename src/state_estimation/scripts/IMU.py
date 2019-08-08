#!/usr/bin/env python

import rospy
import pickle
from os import path
from Adafruit_BNO055 import BNO055
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Imu

CALIBRATION_FILE_PATH = './src/state_estimation/data/calibration.txt'

bno = None
imu_seq = 0
imu_frame_id = 'imu'

'''
Read from imu and return Imu message.
'''
def read_from_imu():
    ax, ay, az = bno.read_linear_acceleration()
    wx, wy, wz = bno.read_gyroscope()
    w, x, y, z = bno.read_quaternion()

    imu_msg = Imu()

    # Liear acceleration
    imu_msg.linear_acceleration.x = ax
    imu_msg.linear_acceleration.y = ay
    imu_msg.linear_acceleration.z = az
    imu_msg.linear_acceleration_covariance[0] = -1

    # Angular velocity
    imu_msg.angular_velocity.x = wx
    imu_msg.angular_velocity.y = wy
    imu_msg.angular_velocity.z = wz
    imu_msg.angular_velocity_covariance[0] = -1

    # Orientation
    imu_msg.orientation.w = w
    imu_msg.orientation.x = x
    imu_msg.orientation.y = y
    imu_msg.orientation.z = z
    imu_msg.orientation_covariance[0] = -1

    # Attach Header
    self.imu_msg.header.stamp = rospy.Time.now()
    self.seq = imu_seq
    self.frame_id = imu_frame_id
    imu_seq += 1

    return imu_msg

if __name__=="__main__":
    bno = BNO055.BNO055(serial_port='/dev/serial0')
    if not bno.begin():
        raise RuntimeError("Failed to initialize BNO055, check connection.")

    rospy.init_node('IMU_publisher')
    pub = rospy.Publisher('imu_data', Imu, queue_size=1)

    # Get calibration data.
    if path.exists(CALIBRATION_FILE_PATH):
        with open(CALIBRATION_FILE_PATH) as file:
            bno.set_calibration(pickle.load(file))
            rospy.loginfo("Read from calibration file successfully.")
    
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        imu_msg = read_from_imu()
        pub.publish(imu_msg)
        rate.sleep()
