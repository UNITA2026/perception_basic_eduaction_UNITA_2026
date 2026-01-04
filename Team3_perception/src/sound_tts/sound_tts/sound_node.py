#!/usr/bin/env python3
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import pygame


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SoundPlayer:
    def __init__(self, logger, volume: float = 0.8):
        self.logger = logger
        self._inited = False
        self._volume = clamp(volume, 0.0, 1.0)
        self._init_mixer()

    def _init_mixer(self):
        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=1024)
            pygame.mixer.music.set_volume(self._volume)
            self._inited = True
            self.logger.info("pygame.mixer initialized")
        except Exception as e:
            self._inited = False
            self.logger.error(f"pygame.mixer init failed: {e}")

    def stop(self):
        if not self._inited:
            return
        try:
            pygame.mixer.music.stop()
        except Exception:
            pass

    def play(self, file_path: Path) -> bool:
        if not self._inited:
            self.logger.warn("mixer not initialized; retry init")
            self._init_mixer()
            if not self._inited:
                return False

        if not file_path.is_file():
            self.logger.error(f"Audio file not found: {file_path}")
            return False

        try:
            # 재생 중이면 끊고 새로 재생
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()

            pygame.mixer.music.load(str(file_path))
            pygame.mixer.music.play()  # non-blocking
            self.logger.info(f"Play: {file_path.name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to play {file_path}: {e}")
            return False


class SoundNode(Node):
    """
    Subscribe:
      - /ultrasonic (std_msgs/Float32MultiArray)

    Behavior (default):
      - data[0..5] are distances (cm)
      - if any sensor distance <= threshold_cm, play sonic_<index+1>.mp3
      - cooldown to prevent spamming
    """

    def __init__(self):
        super().__init__("sound_node")

        # mp3 고정 폴더 (네가 준 경로)
        self.sound_dir = Path("/home/jkm/unita_ws/src/sound_tts/sound")

        # 파라미터 (원하면 launch에서 바꿀 수 있음)
        self.declare_parameter("sub_topic", "/ultrasonic")
        self.declare_parameter("threshold_cm", 5.0)   # 이 거리 이하면 경고음
        self.declare_parameter("cooldown_sec", 1.0)    # 같은 소리 연속 재생 방지
        self.declare_parameter("volume", 0.8)
        self.declare_parameter("pick_mode", "nearest") # "nearest" or "first"

        sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.threshold_cm = float(self.get_parameter("threshold_cm").get_parameter_value().double_value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").get_parameter_value().double_value)
        volume = float(self.get_parameter("volume").get_parameter_value().double_value)
        self.pick_mode = self.get_parameter("pick_mode").get_parameter_value().string_value.strip().lower()

        self.player = SoundPlayer(self.get_logger(), volume=volume)

        self.sub = self.create_subscription(
            Float32MultiArray,
            sub_topic,
            self._on_ultrasonic,
            10
        )

        # 센서별 마지막 재생 시각(스팸 방지)
        self.last_play_time = [0.0] * 6
        self.get_logger().info(f"Subscribed: {sub_topic} (Float32MultiArray)")
        self.get_logger().info(f"Sound dir fixed: {self.sound_dir}")
        self.get_logger().info(f"threshold_cm={self.threshold_cm}, cooldown_sec={self.cooldown_sec}, pick_mode={self.pick_mode}")

    def _sound_path_for_sensor(self, idx_0_based: int) -> Path:
        # idx: 0..5 -> sonic_1..6
        return self.sound_dir / f"sonic_{idx_0_based + 1}.mp3"

    def _choose_sensor_to_play(self, data):
        """
        data: list[float] length>=6 expected
        returns: idx (0..5) or None
        """
        # 유효한 값만 고려 (-1이면 timeout이란 의미로 많이 씀)
        candidates = []
        for i in range(6):
            d = float(data[i])
            if d < 0:
                continue
            if d <= self.threshold_cm:
                candidates.append((d, i))

        if not candidates:
            return None

        if self.pick_mode == "first":
            # 1번부터 6번까지 중 처음 걸리는 것
            return sorted(candidates, key=lambda x: x[1])[0][1]

        # 기본: nearest (가장 가까운 센서)
        return sorted(candidates, key=lambda x: x[0])[0][1]

    def _can_play(self, idx: int) -> bool:
        now = time.time()
        return (now - self.last_play_time[idx]) >= self.cooldown_sec

    def _mark_played(self, idx: int):
        self.last_play_time[idx] = time.time()

    def _on_ultrasonic(self, msg: Float32MultiArray):
        data = msg.data
        if data is None or len(data) < 6:
            self.get_logger().warn(f"Invalid ultrasonic data length: {0 if data is None else len(data)}")
            return

        idx = self._choose_sensor_to_play(data)
        if idx is None:
            return

        if not self._can_play(idx):
            return

        path = self._sound_path_for_sensor(idx)
        ok = self.player.play(path)
        if ok:
            self._mark_played(idx)

    def destroy_node(self):
        try:
            self.player.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
