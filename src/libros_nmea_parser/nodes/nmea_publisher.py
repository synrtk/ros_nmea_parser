import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
from libros_nmea_parser.driver import Ros2NMEADriver

class NMEAProcessor(Node):
    def __init__(self):
        super().__init__('nmea_processor')

        self.driver = Ros2NMEADriver()

        self.serial_port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.serial_baud = self.declare_parameter('baud', 9600).value
        self.frame_id = self.driver.get_frame_id()
        
        self.json_pub = self.create_publisher(String, 'nmea_data_json', 10)

        try:
            self.GPS = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        except serial.SerialException as ex:
            self.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
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
                    self.json_pub.publish(json_msg)
                    self.get_logger().info('Published JSON data: %s' % json_data)
                else:
                    self.get_logger().warn('No valid data returned from add_sentence.')

        except Exception as e:
            self.get_logger().error("Error processing NMEA data: {0}".format(e))

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
