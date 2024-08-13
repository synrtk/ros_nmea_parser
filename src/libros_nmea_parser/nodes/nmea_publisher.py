#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import json
from libros_nmea_parser.driver import RosNMEADriver

class NMEAProcessor:
    def __init__(self):
        rospy.init_node('nmea_processor', anonymous=True)

        self.driver = RosNMEADriver()

        self.serial_port = rospy.get_param('~port', '/dev/ttyACM0')
        self.serial_baud = rospy.get_param('~baud', 9600)
        self.frame_id = self.driver.get_frame_id()
        
        self.json_pub = rospy.Publisher('nmea_data_json', String, queue_size=10)

        # 시리얼 포트 열기
        try:
            self.GPS = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        except serial.SerialException as ex:
            rospy.logfatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
            raise

        # 타이머 설정 (1초마다 process_nmea_data 호출)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.process_nmea_data)

    def process_nmea_data(self, event):
        try:
            if self.GPS.in_waiting:
                data = self.GPS.readline().strip().decode("ascii")
                valid_data = self.driver.add_sentence(data, self.frame_id)

                if valid_data:
                    json_data = self.convert_to_json(valid_data)

                    json_msg = String()
                    json_msg.data = json_data
                    self.json_pub.publish(json_msg)
                    rospy.loginfo('Published JSON data: %s' % json_data)
                else:
                    rospy.logwarn('No valid data returned from add_sentence.')

        except Exception as e:
            rospy.logerr("Error processing NMEA data: {0}".format(e))

    def convert_to_json(self, data):
        return json.dumps(data)

def main():
    try:
        node = NMEAProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'GPS' in locals() and self.GPS.is_open:
            self.GPS.close()

if __name__ == '__main__':
    main()
