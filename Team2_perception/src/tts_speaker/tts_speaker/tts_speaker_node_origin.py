#!/usr/bin/env python3
#=====================================================#
# 기능: ROS2 Traffic Light Speaker (YOLO 신호 연동)
# - /red, /yellow, /green (std_msgs/Bool) 토픽을 구독하여
#   신호(감지/판단)에 맞는 mp3를 재생
#
# 기본 동작(mode=edge):
# - 신호 상태가 바뀔 때만 1회 재생 (중복 재생 방지)
#
# 옵션 동작(mode=level):
# - 특정 신호가 유지되는 동안, 파일을 '끝까지' 재생하고
#   끝난 뒤에도 신호가 유지되면 다음 회차를 이어서 재생
#
# 사운드 파일:
# - sounds/red.mp3, sounds/yellow.mp3, sounds/green.mp3
#
#=====================================================#

import os
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool
from pygame import mixer


class TrafficLightSpeaker(Node):
    def __init__(self):
        super().__init__('traffic_light_speaker_node')

        # -------- Audio init --------
        try:
            mixer.init()
        except Exception as e:
            self.get_logger().error(f"[TLSpeaker] mixer.init() failed: {e}")

        # -------- Parameters --------
        # 사운드 경로 (비우면 cwd/sounds)
        self.declare_parameter('sound_path', '')

        # 토픽 이름(요구사항: red/yellow/green)
        self.declare_parameter('topic_red', 'red')
        self.declare_parameter('topic_yellow', 'yellow')
        self.declare_parameter('topic_green', 'green')

        # 파일 이름
        self.declare_parameter('file_red', 'red.mp3')
        self.declare_parameter('file_yellow', 'yellow.mp3')
        self.declare_parameter('file_green', 'green.mp3')

        # 동작 모드:
        # - "edge": 상태 변화 시 1회 재생(기본)
        # - "level": 해당 상태가 유지되는 동안 반복 재생
        self.declare_parameter('mode', 'edge')

        # 상태가 바뀌면 현재 재생을 끊고 즉시 새 음성 재생할지
        self.declare_parameter('interrupt_on_change', True)

        # 신호 깜빡임 완화: 마지막 True 이후 hold_sec 동안은 "활성"으로 간주
        self.declare_parameter('hold_sec', 0.5)

        # edge 모드에서 너무 잦은 재생 방지(초)
        self.declare_parameter('cooldown_sec', 0.8)

        # -------- Load params --------
        self.sound_path = self.get_parameter('sound_path').get_parameter_value().string_value
        if not self.sound_path:
            self.sound_path = os.path.join(os.getcwd(), 'sounds')

        self.topic_red = self.get_parameter('topic_red').get_parameter_value().string_value
        self.topic_yellow = self.get_parameter('topic_yellow').get_parameter_value().string_value
        self.topic_green = self.get_parameter('topic_green').get_parameter_value().string_value

        self.file_map = {
            'red': self.get_parameter('file_red').get_parameter_value().string_value,
            'yellow': self.get_parameter('file_yellow').get_parameter_value().string_value,
            'green': self.get_parameter('file_green').get_parameter_value().string_value,
        }

        self.mode = self.get_parameter('mode').get_parameter_value().string_value.strip().lower()
        if self.mode not in ('edge', 'level'):
            self.get_logger().warn(f"[TLSpeaker] Invalid mode '{self.mode}', fallback to 'edge'")
            self.mode = 'edge'

        self.interrupt_on_change = bool(
            self.get_parameter('interrupt_on_change').get_parameter_value().bool_value
        )
        self.hold_sec = float(self.get_parameter('hold_sec').get_parameter_value().double_value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').get_parameter_value().double_value)

        # -------- QoS --------
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # -------- State tracking --------
        # 마지막으로 True를 받은 시각(ROS time)
        self.last_seen: Dict[str, Optional[float]] = {'red': None, 'yellow': None, 'green': None}

        # 현재 "활성"으로 판단된 색 (None 가능)
        self.active_color: Optional[str] = None
        # edge 모드에서 마지막으로 "발화(재생 시작)"한 색
        self.last_announced: Optional[str] = None
        # edge 모드 쿨다운 타임스탬프
        self.last_play_time: float = -1e9
        # interrupt_on_change=False일 때, 다음 재생 대기
        self.pending_color: Optional[str] = None

        # -------- Subscribers --------
        self.sub_red = self.create_subscription(Bool, self.topic_red, self._cb_red, qos)
        self.sub_yellow = self.create_subscription(Bool, self.topic_yellow, self._cb_yellow, qos)
        self.sub_green = self.create_subscription(Bool, self.topic_green, self._cb_green, qos)

        # -------- Timer --------
        # - level 모드: 재생이 끝났고 상태가 유지되면 다음 회차 재생
        # - edge 모드: pending_color 처리(재생이 끝났을 때 다음 색 재생)
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(f"[TLSpeaker] sound_path={self.sound_path}")
        self.get_logger().info(f"[TLSpeaker] topics: red='{self.topic_red}', yellow='{self.topic_yellow}', green='{self.topic_green}'")
        self.get_logger().info(f"[TLSpeaker] files: {self.file_map}")
        self.get_logger().info(f"[TLSpeaker] mode={self.mode}, interrupt_on_change={self.interrupt_on_change}, hold_sec={self.hold_sec}, cooldown_sec={self.cooldown_sec}")

    # ----------------- Audio helpers -----------------
    def _file_path(self, color: str) -> str:
        return os.path.join(self.sound_path, self.file_map[color])

    def _is_busy(self) -> bool:
        try:
            return mixer.music.get_busy()
        except Exception:
            return False

    def _stop(self):
        try:
            mixer.music.stop()
        except Exception:
            pass

    def _play_color(self, color: str):
        path = self._file_path(color)
        if not os.path.isfile(path):
            self.get_logger().warn(f"[TLSpeaker] Sound file not found: {path}")
            return

        try:
            mixer.music.load(path)
            mixer.music.play(0)  # 한 번 끝까지 재생
            self.last_play_time = self._now_sec()
            self.get_logger().info(f"[TLSpeaker] PLAY: {color} ({os.path.basename(path)})")
        except Exception as e:
            self.get_logger().error(f"[TLSpeaker] play failed: {e}")

    # ----------------- Time helpers -----------------
    def _now_sec(self) -> float:
        # ROS time (sec as float)
        return self.get_clock().now().nanoseconds * 1e-9

    def _mark_seen(self, color: str, is_true: bool):
        now = self._now_sec()
        if is_true:
            self.last_seen[color] = now
        else:
            # False가 명확히 오면 즉시 비활성 취급되도록 과거로 밀어둠
            # (hold_sec로 깜빡임 완화는 True 기준 유지)
            self.last_seen[color] = None

    def _is_active(self, color: str) -> bool:
        t = self.last_seen.get(color, None)
        if t is None:
            return False
        return (self._now_sec() - t) <= self.hold_sec

    def _resolve_active_color(self) -> Optional[str]:
        # 동시에 여러 개가 활성일 경우 우선순위: red > yellow > green
        if self._is_active('red'):
            return 'red'
        if self._is_active('yellow'):
            return 'yellow'
        if self._is_active('green'):
            return 'green'
        return None

    # ----------------- Subscribers -----------------
    def _cb_red(self, msg: Bool):
        self._mark_seen('red', bool(msg.data))
        self._update_state()

    def _cb_yellow(self, msg: Bool):
        self._mark_seen('yellow', bool(msg.data))
        self._update_state()

    def _cb_green(self, msg: Bool):
        self._mark_seen('green', bool(msg.data))
        self._update_state()

    # ----------------- Core logic -----------------
    def _update_state(self):
        new_color = self._resolve_active_color()
        if new_color == self.active_color:
            return

        self.get_logger().info(f"[TLSpeaker] STATE: {self.active_color} -> {new_color}")
        self.active_color = new_color

        if self.mode == 'edge':
            self._handle_edge_mode_change(new_color)
        else:
            # level 모드는 tick에서 반복재생 담당
            # 상태 바뀌면 즉시 바꾸고 싶으면 여기서 interrupt 처리
            if self.interrupt_on_change:
                self._stop()
                if new_color is not None:
                    self._play_color(new_color)

    def _handle_edge_mode_change(self, new_color: Optional[str]):
        # None으로 바뀌면: (선호에 따라) 재생 중단할지 선택 가능
        # 여기서는 "신호가 없어졌을 때"는 굳이 stop 하지 않음(기존 음성 끝까지)
        # 즉시 끊고 싶으면 아래 주석 해제:
        # if new_color is None and self.interrupt_on_change:
        #     self._stop()
        #     return

        if new_color is None:
            return

        # 쿨다운(너무 빠른 상태 스위칭에 의한 연속 재생 방지)
        if (self._now_sec() - self.last_play_time) < self.cooldown_sec:
            # 너무 빨라서 지금은 재생하지 않고 pending으로만 예약
            self.pending_color = new_color
            return

        if self.interrupt_on_change:
            self._stop()
            self._play_color(new_color)
            self.last_announced = new_color
            self.pending_color = None
        else:
            # 재생 중이면 끝난 뒤 재생하도록 예약
            if self._is_busy():
                self.pending_color = new_color
            else:
                self._play_color(new_color)
                self.last_announced = new_color
                self.pending_color = None

    def _tick(self):
        busy = self._is_busy()

        # edge 모드: 재생이 끝났고 pending이 있으면 재생
        if self.mode == 'edge':
            if (not busy) and (self.pending_color is not None):
                # pending이 현재 상태와 다르면 최신 상태로 교체
                current = self.active_color
                if current is None:
                    # 현재 상태가 없다면 pending은 폐기
                    self.pending_color = None
                    return

                # 쿨다운 체크
                if (self._now_sec() - self.last_play_time) >= self.cooldown_sec:
                    self._play_color(current)
                    self.last_announced = current
                    self.pending_color = None
            return

        # level 모드: 상태가 유지되는 동안 반복 재생
        if self.mode == 'level':
            if self.active_color is None:
                return
            if not busy:
                # 파일 끝났는데도 신호가 유지되면 다음 회차 재생
                self._play_color(self.active_color)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightSpeaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TrafficLightSpeaker Node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
