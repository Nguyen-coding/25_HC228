#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo
from hot.msg import StepEvent  # 사용자 정의 메시지

# 단차 인식 노드 
class HorizontalLineDepthDetector(Node):
    def __init__(self):
        super().__init__('horizontal_line_depth_detector')
        self.bridge = CvBridge()
        self.camera_height_m = 0.235
        self.camera_info_received = False
        self.camera_info_sub = self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self.camera_info_callback,10)
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.step_pub = self.create_publisher(StepEvent, '/step_detected', 10)

        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info("HorizontalLineDepthDetector node initialized.")
        self.get_logger().info(f"카메라 높이: {self.camera_height_m*100:.1f}cm")
        
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True

    def image_callback(self, color_msg, depth_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge 변환 실패: {e}")
            return

        if not self.camera_info_received:
            self.get_logger().warn("카메라 내부 파라미터를 아직 받지 못했습니다.")
            return
        # 원본 이미지 복사
        debug_image = color_image.copy()
        
        # 엣지 검출 및 수평선 찾기
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 2)
        edge = cv2.Canny(gray, 50, 150, 3)

        lines = cv2.HoughLinesP(edge, 1, np.pi / 180, 200, minLineLength=50, maxLineGap=10)
        if lines is None:
            cv2.imshow("Detected Lines", debug_image)
            cv2.waitKey(1)
            return

        # 수평선 필터링
        min_angle = -1.0 * (np.pi / 180.0)
        max_angle = 1.0 * (np.pi / 180.0)
        horizontal_lines = []
        merged_lines = []
        height_threshold = 50

        for l in lines:
            x1, y1, x2, y2 = l[0]
            angle = np.arctan2(y2 - y1, x2 - x1)
            if min_angle < angle < max_angle:
                horizontal_lines.append((x1, y1, x2, y2))

        # 유사한 높이의 수평선 병합
        for l1 in horizontal_lines:
            merged = False
            for i, l2 in enumerate(merged_lines):
                if abs(l1[1] - l2[1]) < height_threshold and abs(l1[3] - l2[3]) < height_threshold:
                    l2[0] = min(l2[0], l1[0])
                    l2[1] = (l2[1] + l1[1]) // 2
                    l2[2] = max(l2[2], l1[2])
                    l2[3] = (l2[3] + l1[3]) // 2
                    merged = True
                    break
            if not merged:
                merged_lines.append(list(l1))

        # 각 수평선의 3D 정보 계산 및 표시
        for i, (x1, y1, x2, y2) in enumerate(merged_lines):
            mid_x = (x1 + x2) // 2
            mid_y = (y1 + y2) // 2
            if mid_y >= depth_image.shape[0] or mid_x >= depth_image.shape[1]:
                continue

            # 중앙점의 깊이 값 (mm)
            depth_val = depth_image[mid_y, mid_x]

            if depth_val == 0 or np.isnan(depth_val):
                continue

            # mm → m 변환 (RealSense 뎁스는 기본적으로 mm 단위)
            if depth_image.dtype == np.uint16:
                depth_val = depth_val / 1000.0  # mm → m

            # 3D 좌표 계산 (카메라 좌표계)
            X = (mid_x - self.cx) * depth_val / self.fx
            Y = (mid_y - self.cy) * depth_val / self.fy
            Z = depth_val

            height_from_ground = self.camera_height_m - Y
            
            self.get_logger().info(
                f"[수평선 #{i}] 위치: ({mid_x},{mid_y}) | 깊이: {Z:.2f}m | Y좌표: {Y:.3f}m | 지면 높이: {height_from_ground*100:.1f}cm"
            )

            if  height_from_ground < 0.20:
                msg = StepEvent()
                msg.height = float(height_from_ground)
                self.step_pub.publish(msg)
                self.get_logger().info(f"[StepEvent] 단차 감지 → 높이: {height_from_ground:.3f} m → 메시지 발행 완료")

            # 수평선 그리기 (파란색)
            cv2.line(debug_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # 중앙점 표시 (초록색)
            cv2.circle(debug_image, (mid_x, mid_y), 5, (0, 255, 0), -1)
            
            # 높이 정보 표시 (흰색)
            cv2.putText(debug_image, 
                      f"{height_from_ground*100:.1f}cm", 
                      (mid_x+10, mid_y),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # 깊이 정보 표시 (노란색)
            cv2.putText(debug_image, 
                      f"Z:{Z:.2f}m", 
                      (mid_x+10, mid_y+25),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Detected Lines", debug_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HorizontalLineDepthDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()