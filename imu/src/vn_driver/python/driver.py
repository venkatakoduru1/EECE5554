#!/usr/bin/env python3


import rospy
import sys
import utm
import time
import serial
from vn_driver.msg import Vectornav 
from std_msgs.msg import Header
import numpy as np
import time
import math
msg = Vectornav()

def is_vnymr_in_string(input_string):
    if(input_string[0]=="$VNYMR"):
        print("VNYMR is there in the string")
    else:
        print("VNYMR is not in the string")
    return 0

"""
Parse VNYMR string into accelerometer, gyroscope, orientation, and magnetometer data.

Args:
    vnymr_data (str): VNYMR data string.

Returns:
    tuple: Parsed data (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
    roll, pitch, yaw, mag_x, mag_y, mag_z).
"""


def parse_vnymr(vnymr_data):
    
    # Remove checksum at the end of the string
    vnymr_data_without_checksum = vnymr_data.split('*')[0]
    
    # Parse VNYMR data string
    parts = vnymr_data_without_checksum.split(',')
    if len(parts) != 13 or parts[0] != '$VNYMR':
        return None
    
    yaw = float(parts[1])
    pitch = float(parts[2])
    roll = float(parts[3])
    
    mag_x = float(parts[4])
    mag_y = float(parts[5])
    mag_z = float(parts[6])
    
    accel_x = float(parts[7])
    accel_y = float(parts[8])
    accel_z = float(parts[9])
    
    gyro_x = float(parts[10])
    gyro_y = float(parts[11])
    gyro_z = float(parts[12])

    # Convert Euler angles to quaternion
    qx, qy, qz, qw = euler_to_quaternion(yaw, pitch, roll)
    
    return roll, pitch, yaw, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, qx, qy, qz, qw


def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return qx, qy, qz, qw


def setDataRateTo40Hz():
    command = b'$VNWRG,07,40*XX'  # Command to set data rate to 40 Hz
    serialPortAddr = rospy.get_param('~port', '/dev/pts/3')  # Adjust the port to  dev/ttyUSB0 when imu is plugged
    serialPort = serial.Serial(serialPortAddr, 115200)
    serialPort.write(command)


def main():
    """
    Main function to read data from serial port, parse VNYMR string,
    and publish data as ROS message.
    """
    setDataRateTo40Hz()
    print(setDataRateTo40Hz)
    port = rospy.get_param('~port', '/dev/pts/3')  # Adjust the port to '/dev/ttyUSB0' when imu is plugged
    ser = serial.Serial(port, 115200, timeout=1)
    rospy.init_node('imu_driver')
    pub = rospy.Publisher('/imu', Vectornav , queue_size=10)  # Adjust topic and message type as needed
    #rate = rospy.Rate(10)
    #i = 0
    
    while not rospy.is_shutdown():
        vnymr_data = ser.readline().decode().strip() 
        try:
            parsed_data = parse_vnymr(vnymr_data)
        except:
            print("Failed to parse data.")
        
        if parsed_data:
            (roll, pitch, yaw, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, qx, qy, qz, qw) = parsed_data
            
            #print()
            msg.header = Header()
            msg.header.frame_id = 'GPS1_Frame'
            msg.header.stamp = rospy.Time.now()
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz
            msg.imu.orientation.w = qw
            msg.imu.angular_velocity.x = gyro_x
            msg.imu.angular_velocity.y = gyro_y
            msg.imu.angular_velocity.z = gyro_z
            msg.imu.linear_acceleration.x = accel_x
            msg.imu.linear_acceleration.y = accel_y
            msg.imu.linear_acceleration.z = accel_z
            msg.mag_field.magnetic_field.x = mag_x
            msg.mag_field.magnetic_field.y = mag_y
            msg.mag_field.magnetic_field.z = mag_z
            #i+=1
            #print(i)
            rospy.loginfo(msg)
            #start_time=time.time()
            pub.publish(msg)
            #end_time=time.time()
            #duration=end_time - start_time
            #frequency=1/duration if duration>0 else float('inf')
            #print("Publishing at {:.2f} Hz".format(frequency))

        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
