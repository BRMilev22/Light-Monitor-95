#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SerialInterfaceNode(Node):
    """
    Direct interface with the Arduino microcontroller via serial.
    Handles sensor data ingestion and actuator command transmission.
    """
    def __init__(self):
        super().__init__('serial_node')
        
        # Communication Channels
        self.pub = self.create_publisher(Int32, 'light_level', 10)
        self.sub = self.create_subscription(Int32, 'set_threshold', self.set_threshold, 10)
        
        # Serial Hardware Interface
        # Intentionally failing fast if device is missing for immediate feedback
        self.arduino = serial.Serial('/dev/cu.usbmodem101', 115200, timeout=1)
        
        self.create_timer(0.1, self.read_data)
        self.get_logger().info("Serial Interface Online")

    def read_data(self):
        if self.arduino.in_waiting:
            line = self.arduino.readline().decode().strip()
            if line.startswith('LIGHT:'):
                try:
                    val = int(line.split(':')[1])
                    self.pub.publish(Int32(data=val))
                except ValueError:
                    pass

    def set_threshold(self, msg):
        self.arduino.write(f'T:{msg.data}\n'.encode())
        self.get_logger().info(f"Threshold updated: {msg.data}")

def main():
    rclpy.init()
    rclpy.spin(SerialInterfaceNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
