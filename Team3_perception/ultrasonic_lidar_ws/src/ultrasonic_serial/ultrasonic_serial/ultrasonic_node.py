import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'ultrasonic',
            10
        )

        self.ser = serial.Serial(
            port='/dev/ttyUSB1',
            baudrate=115200,
            timeout=1.0
        )

        self.timer = self.create_timer(0.1, self.read_serial)  # 10Hz
        self.get_logger().info('Ultrasonic serial node started')

    def read_serial(self):
        line = self.ser.readline().decode('utf-8').strip()
        if not line:
            return

        try:
            values = []
            for part in line.split(','):
                _, val = part.split(':')
                values.append(float(val))

            msg = Float32MultiArray()
            msg.data = values
            self.publisher_.publish(msg)

        except Exception:
            self.get_logger().warn(f'Parse error: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
