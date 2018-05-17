#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys
from numpy import arctan2, sin, cos

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

import tf

### ------------- ###
### ROS Node Init ###
### ------------- ### 
rospy.init_node("imu_auto_cali_node")
pub = rospy.Publisher('/imu_data', Imu, queue_size=1)
imuMsg = Imu()

# Orientation covariance estimation:
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.000025
]

# Angular velocity covariance estimation:
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

### -------------- ###
### Get parameters ###
### -------------- ### 
# IMU frame
imu_frame = rospy.get_param('~imu_frame', 'IMU_link')

# device port
port = rospy.get_param('~port', '/dev/ttyUSB0')

# baudrate
baudrate = rospy.get_param('~baudrate', 57600)


# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
ser.write('#o0' + chr(13))

#discard old input
#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
discard = ser.readlines() 

#start datastream
ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text
ser.flushInput()
ser.write('#o1' + chr(13))

#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()


### -------------------- ###
### Auto yaw-calibration ###
### -------------------- ### 
rospy.loginfo("Auto yaw calibration...")
count = 0.0
vyaw_sum = 0.0
yaw_rad = 0.0
for x in range(0, 300):
    try:
        line = ser.readline()
        line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
        words = string.split(line,",")    # Fields split
        vyaw_sum = vyaw_sum + float(words[8])
        count = count + 1.0
    except:
	    rospy.logwarn("Error in Sensor values")
	    pass

vyaw_bias = float(vyaw_sum/count)
rospy.loginfo("Bias of Vyaw is(rad): %f", vyaw_bias)

### --------------- ###
### Publishing data ###
### --------------- ### 
rospy.loginfo("Publishing IMU data...")
current = rospy.get_time()
last = current
dt = 0.0
seq = 0

yaw_rad = 0.0
flag_recorded = False

accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
omega_z_ref = 0.0

while not rospy.is_shutdown():
    try:
        line = ser.readline()
        line = line.replace("#YPRAG=","")   # Delete "#YPRAG=" #[roll, pitch, yaw, ax, ay, az, omegax, omegay, omegaz]
        words = string.split(line,",")    # Fields split

        omega_z = float(words[8]) - vyaw_bias 
        acc_x = -float(words[3]) * accel_factor
        acc_y = float(words[4]) * accel_factor
        
        current = rospy.get_time()
        
        if flag_recorded:
            dt = current - last;
            yaw_rad += omega_z_ref * dt
            yaw_rad = arctan2(sin(yaw_rad), cos(yaw_rad))

            q = quaternion_from_euler(0,0,yaw_rad)
            imuMsg.orientation.x = q[0]
            imuMsg.orientation.y = q[1]
            imuMsg.orientation.z = q[2]
            imuMsg.orientation.w = q[3]
            imuMsg.angular_velocity.z = omega_z
            imuMsg.linear_acceleration.x = acc_x
            imuMsg.linear_acceleration.y = acc_y
 
            imuMsg.header.stamp= rospy.Time.now()
            imuMsg.header.frame_id = imu_frame
            imuMsg.header.seq = seq
   
            seq = seq + 1
            pub.publish(imuMsg)

        omega_z_ref = omega_z
        last = current
        flag_recorded = True
        #rospy.loginfo("[debug] yaw(rad): %f", yaw_rad)
        
    except:
	    rospy.logwarn("Error in Sensor values")
	    pass
ser.close
#f.close

