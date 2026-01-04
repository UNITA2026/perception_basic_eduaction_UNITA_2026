#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces_pkg.msg import DetectionArray

class TLBoolBridge(Node):
    def __init__(self):
        super().__init__("traffic_light_bool_bridge")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("hold_sec", 0.3)

        self.det_topic = self.get_parameter("detections_topic").value
        self.hold_sec = float(self.get_parameter("hold_sec").value)

        self.pub_red = self.create_publisher(Bool, "/red", 10)
        self.pub_yellow = self.create_publisher(Bool, "/yellow", 10)
        self.pub_green = self.create_publisher(Bool, "/green", 10)

        self.last = {"red": -1e9, "yellow": -1e9, "green": -1e9}

        self.sub = self.create_subscription(DetectionArray, self.det_topic, self.cb, 10)
        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(f"[TLBoolBridge] subscribe {self.det_topic}, hold_sec={self.hold_sec}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb(self, msg: DetectionArray):
        # detections 중 red/yellow/green 하나라도 있으면 last 갱신(우선순위 red>yellow>green)
        seen = None
        for d in msg.detections:
            if d.class_name in ("red", "yellow", "green"):
                # 가장 위험한 방향(우선순위) 선택
                if seen is None:
                    seen = d.class_name
                else:
                    pr = {"red":3,"yellow":2,"green":1}
                    if pr[d.class_name] > pr[seen]:
                        seen = d.class_name
        if seen:
            self.last[seen] = self.now()

    def tick(self):
        t = self.now()
        red = (t - self.last["red"]) <= self.hold_sec
        yellow = (t - self.last["yellow"]) <= self.hold_sec
        green = (t - self.last["green"]) <= self.hold_sec

        self.pub_red.publish(Bool(data=bool(red)))
        self.pub_yellow.publish(Bool(data=bool(yellow)))
        self.pub_green.publish(Bool(data=bool(green)))

def main():
    rclpy.init()
    n = TLBoolBridge()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
