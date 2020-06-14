#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import time
import smbus
import struct
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis

#so this is the new sensor with which i'm working.
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

#so i have used the code that was already given to us in this unit and then by keeping in mind about my functionalities I have extended this code
#So that my robot is capable to do abject avoidence, I added one more sensor mpu 6050, Addition of gyroscope so that my robot can know it's exact location.

rospy.init_node('scared_robot', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)35*

#So these codes i already learned in this unit how they control the movement of my robot.
def callback(frontdata):
 rospy.loginfo(rospy.get_caller_id() + 'I heard %d', frontdata.data)
 
 if frontdata.data > 20:
  vel_msg = Twist()
  vel_msg.linear.x = 2
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = 0
  pub.publish(vel_msg)

def callback0(frontdata):
 rospy.loginfo(rospy.get_caller_id() + 'I heard %d', frontdata.data)


 if frontdata.data < 20:
  vel_msg = Twist()
  vel_msg.linear.x = -2
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = 0
  pub.publish(vel_msg)

def callback1(leftdata):
 rospy.loginfo(rospy.get_caller_id() + 'I heard %d', leftdata.data)

 if leftdata.data < 20:
  vel_msg = Twist()
  vel_msg.linear.x = -2
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = -2
  pub.publish(vel_msg)

def callback2(rightdata):
 rospy.loginfo(rospy.get_caller_id() + 'I heard %d', rightdata.data)

 if rightdata.data < 20:
  vel_msg = Twist()
  vel_msg.linear.x = -2
  vel_msg.linear.y = 0
  vel_msg.linear.z = 0
  vel_msg.angular.x = 0
  vel_msg.angular.y = 0
  vel_msg.angular.z = -2
  pub.publish(vel_msg)

def listener():
 rospy.Subscriber('front', Int16, callback)
 rospy.Subscriber('front', Int16, callback0)
 rospy.Subscriber('left', Int16, callback1)
 rospy.Subscriber('right', Int16, callback2)
#%% MPU 6050 sensor
#So from here now we are adding code for our new sensor code.

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)

    # Calculate a quaternion representing the orientation
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

    # So there this bit of code will  read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

    # Load up the IMU message
    o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)


temp_pub = None
imu_pub = None

# So now let's jump to the main code 
if __name__ == '__main__' :
    listener()

    rospy.init_node('imu_node')
    
    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)
    
    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')
    
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
    
    vel_pub = rospy.Publisher('temperature', Temperature)
    imu_pub = rospy.Publisher('imu/data', Imu)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()