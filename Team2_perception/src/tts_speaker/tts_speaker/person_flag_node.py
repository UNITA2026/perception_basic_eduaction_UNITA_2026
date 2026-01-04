#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces_pkg.msg import DetectionArray

class PersonFlagNode(Node):
    def __init__(self):
        super().__init__('person_flag_node')

        # params
        self.declare_parameter('detections_topic', 'detections')
        self.declare_parameter('person_topic', 'person')
        self.declare_parameter('min_score', 0.5)

        self.det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.person_topic = self.get_parameter('person_topic').get_parameter_value().string_value
        self.min_score = float(self.get_parameter('min_score').get_parameter_value().double_value)

        self.pub = self.create_publisher(Bool, self.person_topic, 10)
        self.sub = self.create_subscription(DetectionArray, self.det_topic, self._cb, 10)

        self.get_logger().info(
            f"[PersonFlag] sub='{self.det_topic}' (DetectionArray) -> pub='{self.person_topic}' (Bool), min_score={self.min_score}"
        )

    def _cb(self, msg: DetectionArray):
        found = False
        for d in msg.detections:
            # class_name이 'person'이고 score가 임계 이상이면 True
            if d.class_name == 'person' and float(d.score) >= self.min_score:
                found = True
                break

        out = Bool()
        out.data = found
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFlagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
