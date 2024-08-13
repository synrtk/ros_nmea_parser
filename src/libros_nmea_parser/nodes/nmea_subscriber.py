#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String

class NMEASubscriber:
    def __init__(self):
        rospy.init_node('nmea_subscriber', anonymous=True)
        self.subscription = rospy.Subscriber(
            'nmea_data_json',
            String,
            self.listener_callback,
            queue_size=10
        )
        
    def listener_callback(self, msg):
        data_str = msg.data
        try:
            data = json.loads(data_str)
            self.process_nmea_data(data)
        except json.JSONDecodeError:
            rospy.logwarn("Received data is not valid JSON: %s", data_str)
        except Exception as e:
            rospy.logerr("Error processing received data: %s", e)

    def process_nmea_data(self, data):
        if 'RMC' in data:
            data_type = 'RMC'
            data = data['RMC']
        elif 'GGA' in data:
            data_type = 'GGA'
            data = data['GGA']
        else:
            data_type = 'Unknown'
            data = {}

        fix_type = data.get('fix_type', 'N/A')
        latitude = data.get('latitude', 'N/A')
        latitude_direction = data.get('latitude_direction', 'N/A')
        longitude = data.get('longitude', 'N/A')
        longitude_direction = data.get('longitude_direction', 'N/A')
        altitude = data.get('altitude', 'N/A')
        mean_sea_level = data.get('mean_sea_level', 'N/A')
        hdop = data.get('hdop', 'N/A')
        speed = data.get('speed', 'N/A')
        num_satellites = data.get('num_satellites', 'N/A')
        utc_time = data.get('utc_time', 'N/A')
        fix_valid = data.get('fix_valid', 'N/A')

        rospy.loginfo(f'Data_Type: {data_type}')
        rospy.loginfo(f'Fix Type: {fix_type}')
        rospy.loginfo(f'Latitude: {latitude} {latitude_direction}')
        rospy.loginfo(f'Longitude: {longitude} {longitude_direction}')
        rospy.loginfo(f'Altitude: {altitude} meters')
        rospy.loginfo(f'Mean Sea Level: {mean_sea_level} meters')
        rospy.loginfo(f'Speed: {speed}')
        rospy.loginfo(f'HDOP: {hdop}')
        rospy.loginfo(f'Number of Satellites: {num_satellites}')
        rospy.loginfo(f'UTC Time: {utc_time}')
        rospy.loginfo(f'Fix Valid: {fix_valid}')

def main():
    nmea_subscriber = NMEASubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
