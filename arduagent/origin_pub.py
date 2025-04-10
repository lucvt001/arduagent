import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class OriginGPSPublisher(Node):
    def __init__(self):
        super().__init__('origin_gps_publisher')

        # Get parameters
        input_topic = self.declare_parameter('input_topic', 'core/gps').get_parameter_value().string_value
        output_topic = self.declare_parameter('output_topic', 'origin_gps').get_parameter_value().string_value

        # Initialize variables
        self.origin_gps = None

        # Subscriber to the input topic
        self.gps_sub = self.create_subscription(NavSatFix, input_topic, self.gps_callback, 1)

        # Publisher to the output topic
        self.gps_pub = self.create_publisher(NavSatFix, output_topic, 10)

        # Timer to publish the origin GPS at the specified rate
        self.timer = self.create_timer(2.0, self.timer_callback)

    def gps_callback(self, msg: NavSatFix):
        # Do nothing if the origin GPS has already been set
        if self.origin_gps is not None:
            return
        
        # Check if the GPS data is valid
        lat, lon = msg.latitude, msg.longitude
        if abs(lat) < 1e-5 and abs(lon) < 1e-5:
            return
        
        self.origin_gps = msg
        self.get_logger().info(f"Origin GPS set: [lat: {msg.latitude}, lon: {msg.longitude}, alt: {msg.altitude}]")

    def timer_callback(self):
        # Publish the origin GPS if it has been set
        if self.origin_gps is not None:
            self.gps_pub.publish(self.origin_gps)


def main(args=None):
    rclpy.init(args=args)
    node = OriginGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()