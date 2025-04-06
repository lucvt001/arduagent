import time
import rclpy
from .base import ArduBase
from std_msgs.msg import String, Float32MultiArray, Float32

class ManualControl:
    def __init__(self, throttle=0, steering=0):
        self.throttle = throttle
        self.steering = steering

class Rover(ArduBase):
    
    def __init__(self):
        super().__init__()

        # Initialize manual control timer
        self.manual_control_signal = ManualControl()
        self.manual_control_timer = self.create_timer(1/self.declare_parameter("manual_control.rate", 5.0).get_parameter_value().double_value, self.manual_control_callback)
        self.last_throttle_control_time, self.last_steering_control_time = self.get_clock().now(), self.get_clock().now()
        self.throttle_timeout = self.declare_parameter("throttle.timeout", 1.0).get_parameter_value().double_value
        self.steering_timeout = self.declare_parameter("steering.timeout", 1.0).get_parameter_value().double_value

    def initialize_subscribers(self):
        buffer = 2
        self.steering_sub = self.create_subscription(
            Float32, 
            self.declare_parameter("subscribers.steering.topic", "control/steering").get_parameter_value().string_value, 
            self.steering_callback, buffer
        )
        self.throttle_sub = self.create_subscription(
            Float32, 
            self.declare_parameter("subscribers.throttle.topic", "control/throttle").get_parameter_value().string_value, 
            self.throttle_callback, buffer
        )
        # self.get_logger().info("Rover subscribers initialized")

    def manual_control(self, steering, throttle):
        return super().manual_control(x=0, y=steering, z=throttle, r=0)
    
    def manual_control_callback(self):
        # Check if the control signal is expired
        time_now = self.get_clock().now()
        if time_now - self.last_throttle_control_time > self.throttle_timeout:
            self.manual_control_signal.throttle = 0
        if time_now - self.last_steering_control_time > self.steering_timeout:
            self.manual_control_signal.steering = 0

        steering = int(self.manual_control_signal.steering * 1000)
        throttle = int(self.manual_control_signal.throttle * 1000)
        self.manual_control(steering, throttle)

    def steering_callback(self, msg):
        self.manual_control_signal.steering = msg.data
        self.last_throttle_control_time = self.get_clock().now()
        
    def throttle_callback(self, msg):
        self.manual_control_signal.throttle = msg.data
        self.last_throttle_control_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    rover = Rover()
    rclpy.spin(rover)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    