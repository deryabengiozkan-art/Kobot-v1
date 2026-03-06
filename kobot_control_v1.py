import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial
import struct
import math

class KobotControlV1(Node):
    def __init__(self):
        super().__init__('kobot_control_v1')
        
        # --- 1. SETTINGS / PARAMETERS ---
        self.declare_parameter('wheel_base', 0.5)      # Distance between wheels (meters)
        self.declare_parameter('wheel_diameter', 0.04) # 4cm diameter (meters)
        self.declare_parameter('ppr', 600.0)           # Pulses per revolution
        self.declare_parameter('serial_port', '/dev/ttyACM0')

        self.L = self.get_parameter('wheel_base').value
        self.D = self.get_parameter('wheel_diameter').value
        self.PPR = self.get_parameter('ppr').value
        
        # Calculation factor: converts m/s to pulses/s
        self.m_s_to_pulse = self.PPR / (math.pi * self.D)

        # --- 2. SERIAL SETUP ---
        port = self.get_parameter('serial_port').value
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.01)
            self.get_logger().info(f'Connected to Pico on {port}')
        except Exception as e:
            self.get_logger().error(f'Serial Error: {e}')
            self.ser = None

        # --- 3. ROS TOPICS ---
        # Listens to: cmd_vel (from network or local)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Publishes to: Battery and Velocity Feedback
        self.bat_pub = self.create_publisher(Float32, 'robot/battery', 10)
        self.feedback_pub = self.create_publisher(Twist, 'robot/velocity_feedback', 10)

        # Timer to read UART from Pico (20Hz)
        self.create_timer(0.05, self.update_telemetry)

    def cmd_callback(self, msg):
        """Converts Twist (m/s) to Binary Pulses and sends to Pico."""
        if self.ser is None: return

        # Differential drive math
        v_l_ms = msg.linear.x - (msg.angular.z * self.L / 2.0)
        v_r_ms = msg.linear.x + (msg.angular.z * self.L / 2.0)

        # Convert to Pulse/s
        p_l = float(v_l_ms * self.m_s_to_pulse)
        p_r = float(v_r_ms * self.m_s_to_pulse)

        # Pack into Binary Struct (ID: 0x01)
        # Format: < (Little-endian), f (float), f (float)
        payload = struct.pack('<ff', p_l, p_r)
        
        # Simple XOR Checksum
        checksum = 0
        for byte in payload: checksum ^= byte
        
        # Frame: Header(2) + ID(1) + Data(8) + Checksum(1) + Footer(1) = 13 Bytes
        packet = b'\xAA\x55\x01' + payload + struct.pack('<B', checksum) + b'\x0A'
        self.ser.write(packet)

    def update_telemetry(self):
        """Reads Binary from Pico and publishes as ROS topics."""
        # 17 Bytes expected: Header(2) + ID(1) + 3 Floats(12) + CS(1) + Footer(1)
        if self.ser and self.ser.in_waiting >= 17:
            if self.ser.read(2) == b'\xAA\x55':
                msg_id = ord(self.ser.read(1))
                raw_data = self.ser.read(12) # Reads 3 floats: p_l, p_r, battery
                cs_received = ord(self.ser.read(1))
                self.ser.read(1) # Read Footer (0x0A)

                # Validate Checksum
                cs_calculated = 0
                for byte in raw_data: cs_calculated ^= byte

                if cs_calculated == cs_received and msg_id == 0x02:
                    p_l_fb, p_r_fb, battery_v = struct.unpack('<fff', raw_data)

                    # Convert back to m/s for ROS network
                    v_l_fb = p_l_fb / self.m_s_to_pulse
                    v_r_fb = p_r_fb / self.m_s_to_pulse

                    # Publish Velocity Feedback (Twist)
                    fb_msg = Twist()
                    fb_msg.linear.x = (v_r_fb + v_l_fb) / 2.0
                    fb_msg.angular.z = (v_r_fb - v_l_fb) / self.L
                    self.feedback_pub.publish(fb_msg)

                    # Publish Battery Voltage
                    bat_msg = Float32()
                    bat_msg.data = battery_v
                    self.bat_pub.publish(bat_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KobotControlV1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
