from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import time
from math import degrees
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32
from sensor_msgs.msg import BatteryState, NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, SetBool
from tf2_ros import TransformBroadcaster

class ArduBase(Node):
    
    def __init__(self):
        '''
        Address: port to be connected to. 
        Connection is automatically done upon initialization.'''
        # Initialize the node
        super().__init__('ardubase')

        # Connect to ardupilot
        self.declare_parameter('possible_addresses', [
            'udpin:localhost:14550',
            'udpin:localhost:14551',
            'udpin:172.17.0.1:14550',
            '/dev/serial0',
            '/dev/ttyACM0',
            '/dev/ttyACM1',
            '/dev/ttyUSB0',
        ])
        possible_addresses = self.get_parameter("possible_addresses").get_parameter_value().string_array_value
        self.connect(possible_addresses)
        
        # Initialize mavlink specific things
        self.boot_time = time.time()
        self.configure_message_interval()
        self.initialize_class_variables()        

        # Initialize tf2 broadcaster
        self.tf2_broadcaster = TransformBroadcaster(self)
        self.parent_frame = self.declare_parameter("parent_frame", "map")
        self.child_frame = self.declare_parameter("child_frame", "vehicle/base_link")

        # Initialize publishers and subscribers
        self.initialize_publishers()
        self.initialize_subscribers()
        self.initialize_services()

    def initialize_publishers(self):
        buffer = 2
        self.altitude_pub = self.create_publisher(Float32, self.declare_parameter("publishers.altitude", "status/altitude").get_parameter_value().string_value, buffer)
        self.heading_pub = self.create_publisher(Float32, self.declare_parameter("publishers.heading", "status/heading").get_parameter_value().string_value, buffer)
        self.gps_pub = self.create_publisher(NavSatFix, self.declare_parameter("publishers.gps", "status/gps").get_parameter_value().string_value, buffer)
        self.pose_pub = self.create_publisher(PoseStamped, self.declare_parameter("publishers.pose", "status/pose").get_parameter_value().string_value, buffer)
        self.path_pub = self.create_publisher(Path, self.declare_parameter("publishers.path", "status/path").get_parameter_value().string_value, buffer)
        self.battery0_pub = self.create_publisher(BatteryState, self.declare_parameter("publishers.battery0", "status/battery0").get_parameter_value().string_value, buffer)
        self.battery1_pub = self.create_publisher(BatteryState, self.declare_parameter("publishers.battery1", "status/battery1").get_parameter_value().string_value, buffer)
        self.temperature_pub = self.create_publisher(Float32, self.declare_parameter("publishers.temperature", "status/temperature").get_parameter_value().string_value, buffer)
        self.system_state_pub = self.create_publisher(String, self.declare_parameter("publishers.system_state", "status/system_state").get_parameter_value().string_value, buffer)
        self.flight_mode_pub = self.create_publisher(String, self.declare_parameter("publishers.flight_mode", "status/flight_mode").get_parameter_value().string_value, buffer)

    def initialize_subscribers(self):
        # To be implemented in the derived classes
        pass

    def initialize_services(self):
        self.arming_srv = self.create_service(SetBool, self.declare_parameter("services.arm", "cmd/arm").get_parameter_value().string_value, self.arming_callback)
        # self.mode_srv = self.create_service(String, self.declare_parameter("services.set_mode", "cmd/set_mode").get_parameter_value().string_value, self.set_mode_callback)
        
    def connect(self, possible_addresses):
        '''
        Address: port to be connected to. 
        Connection is automatically done upon initialization.'''

        self.connected_address = None
        for address in possible_addresses:
            try:
                self.get_logger().info("Trying %s" % (address))
                self.master = mavutil.mavlink_connection(address)
                if self.master.wait_heartbeat(timeout=1) is not None:
                    self.connected_address = address
                    self.get_logger().info("Connected to %s" % (self.connected_address))
                    break
            except:
                pass
        if self.connected_address is None:
            raise Exception("Could not connect to any of the addresses")
        
    def update_vehicle_status(self):
        try:
            update = self.master.recv_match().to_dict()
        except:
            return

        if update['mavpackettype'] == "GLOBAL_POSITION_INT":
            self.altitude = update['relative_alt'] / 1000     # Original unit is mm
            self.is_altitude_updated = True
        elif update['mavpackettype'] == "ATTITUDE":
            attitude = [update['roll'], update['pitch'], update['yaw']]
            self.heading = degrees(update['yaw'])
            self.angular_velocity = [update['rollspeed'], update['pitchspeed'], update['yawspeed']]
            # Convert orientation from NED to FLU frame
            q_ned = QuaternionBase(attitude)
            q_flu = QuaternionBase([q_ned[0], q_ned[1], -q_ned[2], -q_ned[3]])
            self.orientation = list(q_flu)
            self.is_orientation_updated = True
        elif update['mavpackettype'] == "LOCAL_POSITION_NED":
            self.local_position = [update['x'], -update['y'], -update['z']]     # Minus sign convert from NED to FLU frame
            self.local_velocity = [update['vx'], -update['vy'], -update['vz']]
            self.is_local_pose_updated = True
        elif update["mavpackettype"] == "BATTERY_STATUS":
            if update['id'] == 0:
                self.voltage0 = update['voltages'][0] / 1000     # Original unit is mV
                self.current0 = update['current_battery'] / 100     # Original unit is cA
                self.is_battery0_updated = True
            elif update['id'] == 1:
                self.voltage1 = update['voltages'][0] / 1000 
                self.current1 = update['current_battery'] / 100
                self.is_battery1_updated = True
        elif update["mavpackettype"] == "SCALED_PRESSURE":
            # self.pressure = update['press_abs'] / 1000
            self.temperature = update['temperature'] / 100
            self.is_temperature_updated = True
        elif update["mavpackettype"] == "SERVO_OUTPUT_RAW":
            self.servo_output_raw = update
            self.is_servo_output_updated = True
        elif update["mavpackettype"] == "HEARTBEAT":
            mode = update['custom_mode']
            state = update["system_status"]
            self.is_flight_mode_updated = True
            self.is_system_state_updated = True
            self.update_flight_mode(mode)
            self.update_system_state(state)       

    def update_flight_mode(self, mode):
        for m in self.possible_modes:
            if mode == self.master.mode_mapping()[m]:
                self.flight_mode = m
                return
        self.flight_mode = "UNKNOWN"

    def update_system_state(self, state):
        if state == mavutil.mavlink.MAV_STATE_UNINIT:
            self.system_state = "UNINIT"
        elif state == mavutil.mavlink.MAV_STATE_BOOT:
            self.system_state = "BOOT"
        elif state == mavutil.mavlink.MAV_STATE_CALIBRATING:
            self.system_state = "CALIBRATING"
        elif state == mavutil.mavlink.MAV_STATE_STANDBY:
            self.system_state = "STANDBY"
            self.is_armed = False
        elif state == mavutil.mavlink.MAV_STATE_ACTIVE:
            self.system_state = "ACTIVE"
            self.is_armed = True
        elif state == mavutil.mavlink.MAV_STATE_CRITICAL:
            self.system_state = "CRITICAL"
        elif state == mavutil.mavlink.MAV_STATE_EMERGENCY:
            self.system_state = "EMERGENCY"
        elif state == mavutil.mavlink.MAV_STATE_POWEROFF:
            self.system_state = "POWEROFF"       

    def arming_callback(self, request, response):
        if request.data:
            response.success = self.arm()
        else:
            response.success = self.disarm()
        return response
    
    def set_mode_callback(self, request, response):
        mode = request.data
        if mode not in self.possible_modes:
            response.success = False
            self.get_logger().error("Mode %s is not supported" % (mode))
            return response

        response.success = self.set_mode(mode)
        if response.success:
            self.get_logger().info("Mode changed to %s" % (mode))
        else:
            self.get_logger().error("Failed to change mode to %s" % (mode))
        return response

    def set_mode(self, mode, timeout=2):
        print("Settine mode to %s" % (mode))
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

        t = time.time()
        update = None
        while time.time() - t < timeout:
            try:
                update = self.master.recv_match(type="HEARTBEAT", blocking=False)
            except:
                continue

            if update is None:
                continue
            update = update.to_dict()
            if update["custom_mode"] == mode_id:
                print("%s mode set successfully" % (mode))
                return True
            
        print("Mode change failed")
        return False

    def manual_control(self, x, y, z, r):
        self.master.mav.manual_control_send(
            self.master.target_system, x, y, z, r, 0)
        
    def rc_channel_override(self, channel_id, pwm=1500):
        '''
        1 - pitch, 2 - roll, 3 - throttle, 4 - yaw, 5 - forward, 6 - lateral
        '''
        assert (channel_id >= 1 and channel_id<=18), "RC_channel does not exist"
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.
        
    def set_servo_pwm(self, pin, pwm):
        '''
        pin: the pwm channel that the servo is connected to. Should be between 1 to 8.
        '''
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            pin + 8,      # aux servo pin
            pwm,
            0,0,0,0,0     # unused parameters
        )

    def set_relay_state(self, pin, state):
        '''
        pin: the relay pin that the servo is connected to. Should be between 1 to 8.
        state: 1 for on, 0 for off
        '''
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
            0,            # first transmission of this command
            pin - 1,      # relay index starts from 0, so if it's RELAY2, it is considered as relay 1
            state,
            0,0,0,0,0     # unused parameters
        )
        
    def guided_control(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, ax=0, ay=0, az=0, yaw=0, yaw_rate=0):
        '''Only works in GUIDED mode.'''
        self.master.mav.set_position_target_local_ned_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b000111111000),   
            x, y, z,
            vx, vy, vz,
            ax, ay, az,
            yaw, yaw_rate
        )

    def set_ekf_origin(self, lat, lon, alt):
        '''Set the origin of the EKF. The values should be raw float.'''
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1e3),
            int(1e6 * (time.time() - self.boot_time))
        )

    def return_to_launch(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0
        )
    
    def request_message_interval(self, message_id, frequency_hz=None, interval_us=None):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        if frequency_hz is not None:
            interval_us = 1e6 / frequency_hz
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            interval_us, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )
        
    def arm(self) -> bool:
        self.master.arducopter_arm()
        res = self.motors_armed_wait(timeout=1)
        if res:
            return True
        else:
            print("Pixhawk failed to arm")
            return False

    def disarm(self) -> bool:
        self.master.arducopter_disarm()
        res = self.motors_disarmed_wait(timeout=1)
        if res:
            return True
        else:
            print("Pixhawk failed to disarm")
            return False

    def motors_armed_wait(self, timeout) -> bool:
        # Custom implementation of `motors_armed_wait` to allow for timeout
        t = time.time()
        while time.time() - t < timeout:
            try:
                self.master.wait_heartbeat()
                if self.master.motors_armed():
                    return True
            except:
                pass
        return False
            
    def motors_disarmed_wait(self, timeout) -> bool:
        # Custom implementation of `motors_disarmed_wait` to allow for timeout
        t = time.time()
        while time.time() - t < timeout:
            try:
                self.master.wait_heartbeat()
                if not self.master.motors_armed():
                    return True
            except:
                pass
        return False
    
    def configure_message_interval(self):
        # self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, frequency_hz=20)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, frequency_hz=20)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, frequency_hz=20)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, frequency_hz=10)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, frequency_hz=10)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, frequency_hz=10)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, frequency_hz=1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, frequency_hz=10)

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, interval_us=-1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3, interval_us=-1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION, interval_us=-1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, interval_us=-1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, interval_us=-1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, interval_us=-1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, interval_us=-1)

    def initialize_class_variables(self):
        self.possible_modes = ["MANUAL"]
        self.altitude, self.heading, self.is_altitude_updated = None, None, False
        self.angular_velocity, self.orientation, self.is_orientation_updated = None, None, False
        self.local_position, self.local_velocity, self.is_local_pose_updated = None, None, False
        self.voltage0, self.current0, self.is_battery0_updated = None, None, False
        self.voltage1, self.current1, self.is_battery1_updated = None, None, False
        self.temperature, self.is_temperature_updated = None, False
        self.is_armed, self.system_state, self.is_system_state_updated = False, None, False
        self.flight_mode, self.is_flight_mode_updated = None, False
        self.servo_output_raw, self.is_servo_output_updated = None, False