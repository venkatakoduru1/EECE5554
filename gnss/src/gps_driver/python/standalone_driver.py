#!/usr/bin/env python3

"""
This script reads data from a GNSS receiver over USB serial,
parses the GPGGA string, converts latitude and longitude to UTM,
and publishes the data in a custom ROS message.
"""

import rospy
import serial
import utm
from std_msgs.msg import Header
from gps_driver.msg import Customgps 
import time

def is_gpgga_in_string(input_string):
    """
    Checks if the input string contains the GPGGA identifier.

    Args:
        input_string (str): The input string to be checked.

    Returns:
        bool: True if the input string contains the GPGGA identifier, False otherwise.
    """
    return '$GPGGA' in input_string

def deg_mins_to_deg_dec(lat_or_long):
    """
    Converts latitude or longitude from DDmm.mm to DD.dddd format.

    Args:
        lat_or_long (str): Latitude or longitude in DDmm.mm format.

    Returns:
        float: Latitude or longitude in DD.dddd format.
    """
    lat_or_long = str(lat_or_long)  # Convert to string if it's not already
    deg = int(lat_or_long[:2])  
    mins = float(lat_or_long[2:])  
    deg_dec = mins / 60.0  
    return deg + deg_dec

def lat_long_sign_conversion(lat_or_long, lat_or_long_dir):
    """
    Converts latitude or longitude to signed value based on direction.

    Args:
        lat_or_long (float): Latitude or longitude value.
        lat_or_long_dir (str): Latitude or longitude direction ('N', 'S', 'E', 'W').

    Returns:
        float: Signed latitude or longitude value.
    """
    if lat_or_long_dir in ['S', 'W']:  
        return -1 * lat_or_long  
    else:
        return lat_or_long

def convert_to_utm(latitude_signed, longitude_signed):
    """
    Converts latitude and longitude to UTM coordinates.

    Args:
        latitude_signed (float): Signed latitude value.
        longitude_signed (float): Signed longitude value.

    Returns:
        list: UTM coordinates [UTMEasting, UTMNorthing, UTMZone, UTMLetter].
    """
    utm_vals = utm.from_latlon(latitude_signed, longitude_signed)
    return list(utm_vals)

def utc_to_utc_epoch(utc):
    """
    Converts UTC time to epoch time.

    Args:
        utc (str): UTC time in hhmmss.ss format.

    Returns:
        float: Epoch time.
    """
    utc_in_secs = int(utc[:2]) * 3600 + int(utc[2:4]) * 60 + float(utc[4:])
    return time.time() + utc_in_secs

def main():
    """
    Main function to read data from serial port, parse GPGGA string,
    convert latitude and longitude to UTM, and publish data as ROS message.
    """
    rospy.init_node('gnss_driver')
    port = rospy.get_param('~port', '/dev/pts/6')  
    ser = serial.Serial(port, 4800, timeout=1)
    pub = rospy.Publisher('/gps', Customgps, queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        gpgga_data = ser.readline().decode().strip()
        # print(f'Serial String{ser.readline().decode()}') checkpoint1
        gpggaString= gpgga_data.split(",")
        if gpggaString[0] != "$GPGGA":
            #print("Checked for GPGGA string and did not find it") checkpoint2
            continue
        latitude = float(gpggaString[2])
        
        latitudeDir = str(gpggaString[3])
        longitude = float(gpggaString[4])
        longitudeDir = str(gpggaString[5])
        altitude = float(gpggaString[9])
        hdop = float(gpggaString[8])
        utc = gpggaString[1]
        
        # parsed_data = parse_gpgga(gpgga_data)
        # latitude, longitude, altitude, hdop, utc = parsed_data
        LatitudeDec = deg_mins_to_deg_dec(latitude)
        LongitudeDec = deg_mins_to_deg_dec(longitude)
        LatitudeSigned = lat_long_sign_conversion(LatitudeDec, latitudeDir)
        LongitudeSigned = lat_long_sign_conversion(LongitudeDec, longitudeDir)
        UTMCoords = convert_to_utm(LatitudeSigned, LongitudeSigned)
        epoch_time = utc_to_utc_epoch(utc) 
        #print("Milestone Reached") I am working , I guess
        msg = Customgps()
        msg.header = Header()
        msg.header.frame_id = 'GPS1_Frame'
        msg.header.stamp.secs = int(epoch_time)
        msg.header.stamp.nsecs = int((epoch_time - int(epoch_time)) * 1e9)
        msg.latitude = LatitudeDec
        msg.longitude = LongitudeDec
        msg.altitude = altitude
        msg.utm_easting = UTMCoords[0]
        msg.utm_northing = UTMCoords[1]
        msg.zone = UTMCoords[2]
        msg.letter = UTMCoords[3]
        msg.hdop = hdop
        msg.gpgga_read = gpgga_data
            
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        
        pub.publish(msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
