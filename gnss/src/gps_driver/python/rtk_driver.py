#!/usr/bin/env python3

"""
This script reads data from an RTK GNSS receiver over USB serial,
parses the GNGGA string, converts latitude and longitude to UTM,
and publishes the data in a custom ROS message.
"""

import rospy
import serial
import utm
from std_msgs.msg import Header
from gps_driver.msg import Customrtk  
import time

def is_gngga_in_string(input_string):
    """
    Checks if the input string contains the GNGGA identifier.

    Args:
        input_string (str): The input string to be checked.

    Returns:
        bool: True if the input string contains the GNGGA identifier, False otherwise.
    """
    return '$GNGGA' in input_string

def parse_lat_lon(degrees, minutes, direction):
    degrees_decimal = float(degrees) + float(minutes) / 60.0
    if direction == 'S' or direction == 'W':
        degrees_decimal *= -1
    return degrees_decimal

def parse_gngga(gngga_data):
    """
    Parse GNGGA string into latitude, longitude, altitude, UTC, fix quality, HDOP.

    Args:
        gngga_data (str): GNGGA data string.

    Returns:
        tuple: Parsed data (latitude, longitude, altitude, utc, fix_quality, hdop).
    """
    # Parse GNGGA data string
    parts = gngga_data.split(',')
    if len(parts) < 15 or parts[0] != '$GNGGA':
        return None
    
    latitude_degrees = parts[2][:2]
    latitude_minutes = parts[2][2:]
    latitude_direction = parts[3]
    latitude = parse_lat_lon(latitude_degrees, latitude_minutes, latitude_direction)

    longitude_degrees = parts[4][:3]
    longitude_minutes = parts[4][3:]
    longitude_direction = parts[5]
    longitude = parse_lat_lon(longitude_degrees, longitude_minutes, longitude_direction)

    altitude = float(parts[9]) if parts[9] else None
    utc = parts[1]
    fix_quality = int(parts[6]) if parts[6] else None
    hdop = float(parts[8]) if parts[8] else None
    
    return latitude, longitude, altitude, utc, fix_quality, hdop

def main():
    """
    Main function to read data from serial port, parse GNGGA string,
    convert latitude and longitude to UTM, and publish data as ROS message.
    """
    rospy.init_node('gnss_driver')
    port = rospy.get_param('~port', '/dev/pts/1')  
    ser = serial.Serial(port, 4800, timeout=1)
    pub = rospy.Publisher('/gps', Customrtk, queue_size=10) 
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        gngga_data = ser.readline().decode().strip()  
        if is_gngga_in_string(gngga_data):
            parsed_data = parse_gngga(gngga_data)
            if parsed_data:
                latitude, longitude, altitude, utc, fix_quality, hdop = parsed_data
                
                # Convert latitude and longitude to UTM
                utm_coords = utm.from_latlon(latitude, longitude)
                utm_easting, utm_northing, utm_zone, utm_letter = utm_coords

                msg = Customrtk()
                msg.header = Header()
                msg.header.frame_id = 'GPS1_Frame'
                msg.header.stamp = rospy.Time.now()
                msg.latitude = latitude
                msg.longitude = longitude
                msg.altitude = altitude
                msg.utm_easting = utm_easting
                msg.utm_northing = utm_northing
                msg.zone = str(utm_zone)  # Convert to string
                msg.letter = utm_letter
                msg.fix_quality = fix_quality
                msg.hdop = hdop
                msg.utc = utc
                msg.gngga_read = gngga_data

                rospy.loginfo(msg)
                pub.publish(msg)

        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
