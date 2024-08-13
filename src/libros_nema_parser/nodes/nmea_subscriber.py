import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NMEASubscriber(Node):
    def __init__(self):
        super().__init__('nmea_subscriber')
        self.subscription = self.create_subscription(
            String,
            'nmea_data_json',
            self.listener_callback,
            10
        )
        self.subscription 

    def listener_callback(self, msg):
        data_str = msg.data
        data = json.loads(data_str)  
        self.process_nmea_data(data)

    def process_nmea_data(self, data):
        if 'RMC' in data:
            data_type = 'RMC'
            data = data['RMC']
        elif 'GGA' in data:
            data_type = 'GGA'
            data = data['GGA']
        fix_type = data.get('fix_type')
        latitude = data.get('latitude')
        latitude_direction = data.get('latitude_direction')
        longitude = data.get('longitude')
        longitude_direction = data.get('longitude_direction')
        altitude = data.get('altitude')
        mean_sea_level = data.get('mean_sea_level')
        hdop = data.get('hdop')
        speed = data.get('speed')
        num_satellites = data.get('num_satellites')
        utc_time = data.get('utc_time')
        fix_valid = data.get('fix_valid')

        self.get_logger().info(f'Data_Type: {data_type}')
        self.get_logger().info(f'Fix Type: {fix_type}')
        self.get_logger().info(f'Latitude: {latitude} {latitude_direction}')
        self.get_logger().info(f'Longitude: {longitude} {longitude_direction}')
        self.get_logger().info(f'Altitude: {altitude} meters')
        self.get_logger().info(f'Mean Sea Level: {mean_sea_level} meters')
        self.get_logger().info(f'Speed: {speed}')
        self.get_logger().info(f'HDOP: {hdop}')
        self.get_logger().info(f'Number of Satellites: {num_satellites}')
        self.get_logger().info(f'UTC Time: {utc_time}')

def main(args=None):
    rclpy.init(args=args)
    nmea_subscriber = NMEASubscriber()
    rclpy.spin(nmea_subscriber)
    nmea_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
