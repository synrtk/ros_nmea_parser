import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs 
from libros_nmea_parser.checksum_utils import check_nmea_checksum
from libros_nmea_parser import parser


class Ros2NMEADriver(Node):
    def __init__(self):
        super().__init__('nmea_navsat_driver')

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', True).value
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            self.get_logger().warn("Received a sentence with an invalid checksum. " +
                                   "Sentence was: %s" % nmea_string)
            return False

        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = self.get_clock().now().to_msg()

        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]
            if current_fix.status.status > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = self.euler_to_quaternion(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)
        else:
            return None
        return parsed_sentence

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to a quaternion."""
        # Roll, pitch, yaw in radians
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return (qx, qy, qz, qw)

    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gps').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id
