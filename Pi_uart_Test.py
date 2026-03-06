import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import struct

class UARTTestNode(Node):
    def __init__(self):
        super().__init__('uart_test_node')
        
        # Serial configuration
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        except:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1) # Alternative port

        # Timer for sending "Ping" every 1 second
        self.create_timer(1.0, self.send_ping)
        
        # Timer for reading "Pong" response (50Hz)
        self.create_timer(0.02, self.read_pong)
        
        self.ping_value = 1.23
        self.get_logger().info('UART Test Node started. Sending Pings...')

    def send_ping(self):
        # Prepare Binary Packet (ID: 0x01, 2 Floats)
        payload = struct.pack('<ff', self.ping_value, self.ping_value + 1.0)
        
        # XOR Checksum
        checksum = 0
        for b in payload: checksum ^= b
        
        # Frame Construction
        packet = b'\xAA\x55\x01' + payload + struct.pack('<B', checksum) + b'\x0A'
        self.ser.write(packet)
        self.get_logger().info(f'Sent Ping: {self.ping_value}')

    def read_pong(self):
        if self.ser.in_waiting >= 17: # Header(2) + ID(1) + 3*Float(12) + CS(1) + Footer(1)
            header = self.ser.read(2)
            if header == b'\xAA\x55':
                msg_id = ord(self.ser.read(1))
                raw_data = self.ser.read(12)
                checksum = ord(self.ser.read(1))
                footer = self.ser.read(1)
                
                # Simple validation
                if msg_id == 0x02:
                    val_l, val_r, bat = struct.unpack('<fff', raw_data)
                    self.get_logger().info(f'RECEIVED PONG -> L: {val_l}, R: {val_r}, Bat: {bat}')

def main():
    rclpy.init()
    node = UARTTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
