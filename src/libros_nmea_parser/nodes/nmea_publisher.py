import rclpy
from rclpy.node import Node
from ros_nmea_parser.msg import ParseData
from libros_nmea_parser.driver import Ros2NMEADriver
import serial
import json

class NMEAProcessor(Node):
    def __init__(self):
        super().__init__('nmea_processor')

        self.driver = Ros2NMEADriver()

        self.serial_port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.serial_baud = self.declare_parameter('baud', 9600).value
        self.frame_id = self.driver.get_frame_id()
        
        self.data_pub = self.create_publisher(ParseData, 'nmea_parsed_data', 10)

        try:
            self.GPS = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        except serial.SerialException as ex:
            self.get_logger().fatal(f"Could not open serial port: {ex}")
            raise

        self.timer = self.create_timer(1.0, self.process_nmea_data)

    def process_nmea_data(self):
        try:
            if self.GPS.in_waiting:
                data = self.GPS.readline().strip().decode("ascii")
                valid_data = self.driver.add_sentence(data, self.frame_id)

                if valid_data:
                    json_data = self.convert_to_json(valid_data)
                    json_msg = String()
                    json_msg.data = json_data
                    
                    parsed_msg = ParseData()

                    if 'RMC' in valid_data:
                        data = valid_data['RMC']
                        parsed_msg.data_type = 'RMC'
                        parsed_msg.fix_type = 1  # RMC는 fix_type을 제공하지 않으므로 기본값을 설정합니다.
                        parsed_msg.latitude = data['latitude']
                        parsed_msg.latitude_direction = data['latitude_direction']
                        parsed_msg.longitude = data['longitude']
                        parsed_msg.longitude_direction = data['longitude_direction']
                        parsed_msg.speed = data['speed']
                        parsed_msg.fix_valid = data['fix_valid']
                        parsed_msg.utc_time = data['utc_time']

                    elif 'GGA' in valid_data:
                        data = valid_data['GGA']
                        parsed_msg.data_type = 'GGA'
                        parsed_msg.fix_type = data['fix_type']
                        parsed_msg.latitude = data['latitude']
                        parsed_msg.latitude_direction = data['latitude_direction']
                        parsed_msg.longitude = data['longitude']
                        parsed_msg.longitude_direction = data['longitude_direction']
                        parsed_msg.altitude = data['altitude']
                        parsed_msg.mean_sea_level = data['mean_sea_level']
                        parsed_msg.hdop = data['hdop']
                        parsed_msg.num_satellites = data['num_satellites']
                        parsed_msg.utc_time = data['utc_time']

                    elif 'HDT' in valid_data:
                        data = valid_data['HDT']
                        parsed_msg.data_type = 'HDT'
                        parsed_msg.heading = data['heading']

                    self.data_pub.publish(parsed_msg)
                    self.get_logger().info(f'Published parsed data: {parsed_msg}')

        except Exception as e:
            self.get_logger().error(f"Error processing NMEA data: {e}")

    def convert_to_json(self, data):
        return json.dumps(data)

def main(args=None):
    rclpy.init(args=args)
    node = NMEAProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
