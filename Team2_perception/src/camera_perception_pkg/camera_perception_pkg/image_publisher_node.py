#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import sys, cv2, os, subprocess, time

#---------------Variable Setting---------------
PUB_TOPIC_NAME = 'image_raw'
DATA_SOURCE = 'camera'
CAM_NUM = 2   # 실제 번호 확인
IMAGE_DIRECTORY_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sample_dataset'
VIDEO_FILE_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4'
SHOW_IMAGE = True
TIMER = 0.03
#----------------------------------------------

EXPO_TARGET = 380
GAIN_TARGET = 0
WB_TARGET   = 5500
FPS_TARGET  = 30
WIDTH, HEIGHT = 640, 640

class ImagePublisherNode(Node):
    def __init__(self, data_source=DATA_SOURCE, cam_num=CAM_NUM, img_dir=IMAGE_DIRECTORY_PATH, video_path=VIDEO_FILE_PATH, pub_topic=PUB_TOPIC_NAME, logger=SHOW_IMAGE, timer=TIMER):
        super().__init__('image_publisher_node')
        self.declare_parameter('data_source', data_source)
        self.declare_parameter('cam_num', cam_num)
        self.declare_parameter('img_dir', img_dir)
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        
        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.cam_num = self.get_parameter('cam_num').get_parameter_value().integer_value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.br = CvBridge()

        self.cap = None
        self.enforce_until = 0.0

        if self.data_source == 'camera':
            dev = f"/dev/video{self.cam_num}"

            # 0) 장치가 비워질 때까지 대기(레이스 방지)
            self._wait_device_free(dev, timeout=3.0)

            # 1) 포맷/컨트롤 선적용
            self._apply_cam_controls(dev, verbose=True)

            # 2) 캡처 열기(V4L2 백엔드 + YUYV + 해상도 + FPS)
            self.cap = cv2.VideoCapture(self.cam_num, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, FPS_TARGET)

            # 3) 스트리밍 시작 후 드라이버가 값을 되돌리는 경우 재적용
            time.sleep(0.8)
            self._apply_cam_controls(dev, verbose=False)

            # 4) 시작 후 5초 동안 0.5초 주기 워치독으로 재확인/재적용
            self.enforce_until = time.time() + 5.0
            self.create_timer(0.5, lambda: self._re_enforce_ctrls(dev))

        elif self.data_source == 'video':
            self.cap = cv2.VideoCapture(self.video_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            if not self.cap.isOpened():
                self.get_logger().error(f'Cannot open video file: {self.video_path}')
                rclpy.shutdown(); sys.exit(1)

        elif self.data_source == 'image':
            if os.path.isdir(self.img_dir):
                self.img_list = sorted(os.listdir(self.img_dir))
                self.img_num = 0
            else:
                self.get_logger().error(f'Not a directory file: {self.img_dir}')
                rclpy.shutdown(); sys.exit(1)
        else:
            self.get_logger().error(f"Wrong data source: {self.data_source}")
            rclpy.shutdown(); sys.exit(1)

        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # --- 유틸: 장치 유휴 대기 ---
    def _wait_device_free(self, dev: str, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = subprocess.run(["fuser", "-s", dev])
            if p.returncode != 0:  # 점유자 없음
                return True
            time.sleep(0.1)
        return False

    # --- 유틸: 컨트롤 조회 ---
    def _get_ctrls(self, dev: str):
        out = subprocess.run(
            ["v4l2-ctl","-d",dev,"--get-ctrl=auto_exposure,exposure_time_absolute,gain,exposure_dynamic_framerate"],
            capture_output=True, text=True
        )
        return out.stdout.strip()

    # --- 유틸: 컨트롤 적용(리트라이 포함) ---
    def _apply_cam_controls(self, dev: str, verbose=False):
        try:
            # 포맷 전환 트리거(MJPG→YUYV) 후 최종 YUYV 고정
            subprocess.run(["v4l2-ctl","-d",dev,f"--set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat=MJPG"], check=False)
            subprocess.run(["v4l2-ctl","-d",dev,f"--set-fmt-video=width={WIDTH},height={HEIGHT},pixelformat=YUYV"], check=False)
            subprocess.run(["v4l2-ctl","-d",dev,f"--set-parm={FPS_TARGET}"], check=False)

            # 자동 기능 OFF(수동 노출 모드)
            subprocess.run(
                ["v4l2-ctl","-d",dev,"--set-ctrl=auto_exposure=1,exposure_dynamic_framerate=0,white_balance_automatic=0,focus_automatic_continuous=0"],
                check=False
            )
            # 노출/게인/화이트밸런스 적용(두 번 찍어 강제)
            subprocess.run(["v4l2-ctl","-d",dev,f"--set-ctrl=gain={GAIN_TARGET},white_balance_temperature={WB_TARGET}"], check=False)
            subprocess.run(["v4l2-ctl","-d",dev,f"--set-ctrl=exposure_time_absolute={max(3,EXPO_TARGET-1)}"], check=False)
            time.sleep(0.1)
            subprocess.run(["v4l2-ctl","-d",dev,f"--set-ctrl=exposure_time_absolute={EXPO_TARGET}"], check=False)

            # 검증 & 필요시 한 번 더
            s = self._get_ctrls(dev)
            if verbose:
                self.get_logger().info(f"[v4l2] {s}")
            if (f"auto_exposure: 1" not in s) or (f"exposure_time_absolute: {EXPO_TARGET}" not in s) or (f"gain: {GAIN_TARGET}" not in s):
                time.sleep(0.2)
                subprocess.run(["v4l2-ctl","-d",dev,f"--set-ctrl=exposure_time_absolute={EXPO_TARGET},gain={GAIN_TARGET}"], check=False)
                if verbose:
                    self.get_logger().info(f"[v4l2] re-apply → {self._get_ctrls(dev)}")
        except Exception as e:
            self.get_logger().warn(f"v4l2-ctl failed: {e}")

    # --- 워치독: 시작 후 몇 초간 재확인/재적용 ---
    def _re_enforce_ctrls(self, dev: str):
        if time.time() > self.enforce_until:
            return
        s = self._get_ctrls(dev)
        need = (f"auto_exposure: 1" not in s) or (f"exposure_time_absolute: {EXPO_TARGET}" not in s) or (f"gain: {GAIN_TARGET}" not in s)
        if need:
            self._apply_cam_controls(dev, verbose=False)

    def timer_callback(self):
        if self.data_source == 'camera':
            ret, frame = self.cap.read()
            if ret:
                # 크기 보정
                if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
                    frame = cv2.resize(frame, (WIDTH, HEIGHT))

                # ✅ 수정: encoding 명시 (bgr8)
                image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')

                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame'
                self.publisher.publish(image_msg)
                if self.logger:
                    cv2.imshow('Camera Image', frame)
                    cv2.waitKey(1)

        elif self.data_source == 'image':
            if self.img_num < len(self.img_list):
                img_file = self.img_list[self.img_num]
                img_path = os.path.join(self.img_dir, img_file)
                img = cv2.imread(img_path)
                if img is None:
                    self.get_logger().warn(f'Skipping non-image file: {img_file}')
                else:
                    img = cv2.resize(img, (WIDTH, HEIGHT))

                    # ✅ 수정: encoding 명시 (bgr8)
                    image_msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')

                    image_msg.header = Header()
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'image_frame'
                    self.publisher.publish(image_msg)
                    if self.logger:
                        self.get_logger().info(f'Published image: {img_file}')
                        cv2.imshow('Saved Image', img)
                        cv2.waitKey(1)
                self.img_num += 1
            else:
                self.img_num = 0

        elif self.data_source == 'video':
            ret, img = self.cap.read()
            if ret:
                if img.shape[1] != WIDTH or img.shape[0] != HEIGHT:
                    img = cv2.resize(img, (WIDTH, HEIGHT))

                # ✅ 수정: encoding 명시 (bgr8)
                image_msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')

                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame'
                self.publisher.publish(image_msg)
                if self.logger:
                    cv2.imshow('Video Frame', img)
                    cv2.waitKey(1)
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if getattr(node, 'cap', None) and node.cap.isOpened():
                node.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
