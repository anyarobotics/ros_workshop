#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import struct
import serial
import time

# Based on your working PyQt6 logic
HEADER      = bytes([0xAA, 0x55])
# Packet: header(2s), timestamp(I), then 5 motors (3f, 2B each), then 3B padding/checksum
PKT_FMT     = '<2sIfffBBfffBBfffBBfffBBfffBBB'
PKT_SIZE    = struct.calcsize(PKT_FMT)

class IR52cTelemetryNode(Node):
    def __init__(self):
        super().__init__('ir52c_telemetry')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Configure Serial
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to STM32 on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        self.buf = bytearray()
        self.timer = self.create_timer(0.01, self.spin_once) # 100Hz

    def spin_once(self):
        if self.ser.in_waiting > 0:
            chunk = self.ser.read(256)
            self.buf.extend(chunk)
            
            while True:
                idx = self.buf.find(HEADER)
                if idx < 0: 
                    self.buf = self.buf[-1:]
                    break
                if len(self.buf) - idx < PKT_SIZE: 
                    break
                
                pkt_data = bytes(self.buf[idx : idx + PKT_SIZE])
                self.buf = self.buf[idx + PKT_SIZE:]
                
                parsed = self.parse_packet(pkt_data)
                if parsed and parsed['chk_ok']:
                    self.publish_joint_state(parsed)

    def parse_packet(self, data):
        fields = struct.unpack_from(PKT_FMT, data)
        # Checksum validation (XOR)
        chk = 0
        for b in data[:PKT_SIZE - 1]:
            chk ^= b
            
        motors = []
        for i in range(5):
            motors.append({
                'angle': fields[2 + i*5],
                'state': fields[2 + i*5 + 3]
            })
        return {
            'motors': motors,
            'chk_ok': chk == data[PKT_SIZE - 1]
        }

    def publish_joint_state(self, parsed):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Ensure names match your URDF
        msg.name = ['base', 'shoulder', 'elbow', 'wrist', 'tool']
        
        # Convert degrees (from STM32) to Radians (for ROS 2)
        msg.position = [m['angle'] * (3.14159 / 180.0) for m in parsed['motors']]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IR52cTelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()