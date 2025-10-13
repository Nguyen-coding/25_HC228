#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False


class Mode(Enum):
    STRAIGHT   = 0
    TURN_LEFT  = 1
    TURN_RIGHT = 2
    ZERO_TURN  = 3
    HARD_STOP  = 4  # 안전용 (너무 가까우면 완전 정지)


class AvoidanceController(Node):

    def __init__(self):
        super().__init__('avoidance_controller')

        # =======================
        # Tuning Parameters
        # =======================
        # ROI/거리 임계
        self.declare_parameter('roi_distance', 2.0)           # ROI 반경 [m]
        self.declare_parameter('angle_range_deg', 30.0)        # ±전방 각 [deg]
        self.declare_parameter('zero_turn_distance', 0.6)     # 제로턴 진입 임계 [m]
        self.declare_parameter('zero_turn_exit', 0.8)         # 제로턴 이탈 임계(히스테리시스) [m]
        self.declare_parameter('hard_stop_distance', 0.10)     # 완전 정지 [m]

        # 속도 제한/이득
        self.declare_parameter('v_max', 0.25)                  # 최대 선속도 [m/s]
        self.declare_parameter('v_straight', 0.20)             # 직진시 선속도 [m/s]
        self.declare_parameter('v_turn', 0.12)                 # 좌/우 회전시 선속도 [m/s]
        self.declare_parameter('w_max', 2.0)                   # 최대 각속도 [rad/s]
        self.declare_parameter('w_zero_max', 2.5)              # 제로턴 최대 각속도 [rad/s]
        self.declare_parameter('w_turn_min', 0.5)              # 좌/우 회전시 최소 각속도 [rad/s]
        self.declare_parameter('w_zero_min', 0.6)              # 제로턴 최소 각속도 [rad/s]
        self.declare_parameter('kp_ang', 2.5)                  # 일반 주행 회전 P이득
        self.declare_parameter('kp_ang_zero', 3.0)             # 제로턴 회전 P이득

        # 상태 결정용 에러 임계 (히스테리시스)
        self.declare_parameter('turn_entry_err', 0.12)         # |err| ≥ 진입 → 좌/우 회전
        self.declare_parameter('turn_exit_err', 0.08)          # |err| ≤ 이탈 → 직진
        self.declare_parameter('error_alpha', 0.2)             # err 평활화 (0~1)

        # 발행주기
        self.declare_parameter('publish_rate_hz', 20.0)

        # 부호 뒤집기(방향 반대로 돌면 true)
        self.declare_parameter('invert_angular', False)

        # 디버그
        self.declare_parameter('show_debug', True)             # OpenCV 시각화
        self.declare_parameter('log_period_s', 1.0)            # 일반 로그 간격

        # (STM32에서 아직 1.0 스케일이 아니라면 임시 보정)
        self.declare_parameter('SCALE_V', 1.0)                 # m/s 스케일
        self.declare_parameter('SCALE_W', 1.0)                 # rad/s 스케일

        # 라이다 정면 오프셋(필요시 기구 오차 보정)
        self.declare_parameter('front_offset_deg', 315.0)

        # === 전방(센터) 밴드/감속 파라미터 ===
        self.declare_parameter('front_band_deg', 10.0)         # 전방 밴드 반각 [deg]
        self.declare_parameter('slowdown_distance', 1.20)      # 이보다 가까우면 감속 시작 [m]
        self.declare_parameter('v_slow', 0.08)                 # 감속시 최소 선속도 [m/s]

        # 파라미터 읽기
        self.roi_distance      = float(self.get_parameter('roi_distance').value)
        self.angle_range_deg   = float(self.get_parameter('angle_range_deg').value)
        self.zero_turn_dist    = float(self.get_parameter('zero_turn_distance').value)
        self.zero_turn_exit    = float(self.get_parameter('zero_turn_exit').value)
        self.hard_stop_dist    = float(self.get_parameter('hard_stop_distance').value)

        self.v_max             = float(self.get_parameter('v_max').value)
        self.v_straight        = float(self.get_parameter('v_straight').value)
        self.v_turn            = float(self.get_parameter('v_turn').value)
        self.w_max             = float(self.get_parameter('w_max').value)
        self.w_zero_max        = float(self.get_parameter('w_zero_max').value)
        self.w_turn_min        = float(self.get_parameter('w_turn_min').value)
        self.w_zero_min        = float(self.get_parameter('w_zero_min').value)
        self.kp_ang            = float(self.get_parameter('kp_ang').value)
        self.kp_ang_zero       = float(self.get_parameter('kp_ang_zero').value)

        self.turn_entry_err    = float(self.get_parameter('turn_entry_err').value)
        self.turn_exit_err     = float(self.get_parameter('turn_exit_err').value)
        self.error_alpha       = float(self.get_parameter('error_alpha').value)
        self.publish_rate_hz   = float(self.get_parameter('publish_rate_hz').value)
        self.invert_angular    = bool(self.get_parameter('invert_angular').value)

        self.show_debug        = bool(self.get_parameter('show_debug').value)
        self.log_period_s      = float(self.get_parameter('log_period_s').value)

        self.SCALE_V           = float(self.get_parameter('SCALE_V').value)
        self.SCALE_W           = float(self.get_parameter('SCALE_W').value)

        self.front_offset_deg  = float(self.get_parameter('front_offset_deg').value)
        self.front_band_deg    = float(self.get_parameter('front_band_deg').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.v_slow            = float(self.get_parameter('v_slow').value)

        # 내부 상태
        self.error = 0.0             # 비정규화 오차(픽셀 유사 단위)
        self.err_f = 0.0             # 평활화된 정규화 오차(-1~+1)
        self.min_distance_R = float('inf')
        self.min_distance_L = float('inf')
        self.min_distance_F = float('inf')   # 전방(센터) 최소거리
        self._last_log = time.time()

        # 상태 머신
        self.mode: Mode = Mode.STRAIGHT
        self._last_mode = self.mode
        self._mode_changed_ts = time.time()

        # 시각화용 캔버스 설정
        self.IMG_W = 600
        self.IMG_H = 600
        self.cx = self.IMG_W // 2
        self.cy = self.IMG_H // 2

        # ROS I/O
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_cmd)

        # 런타임 파라미터 반영
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info('AvoidanceController node started (SI units for /cmd_vel).')

    # 런타임 파라미터 업데이트
    def _on_param_change(self, params):
        for p in params:
            if p.name == 'front_offset_deg':self.front_offset_deg= float(p.value)
            elif p.name == 'angle_range_deg':   self.angle_range_deg = float(p.value)
            elif p.name == 'roi_distance':    self.roi_distance    = float(p.value)
            
            elif p.name == 'front_band_deg':  self.front_band_deg  = float(p.value)
            elif p.name == 'slowdown_distance': self.slowdown_distance = float(p.value)
            elif p.name == 'v_slow':          self.v_slow          = float(p.value)
        return SetParametersResult(successful=True)

    # ---------------------------
    # 각도 [-180, +180) 보정 함수
    # --------------------------
    @staticmethod
    def _wrap180(deg: float) -> float:
        return ((deg + 180.0) % 360.0) - 180.0

    def scan_callback(self, scan: LaserScan):
        draw = self.show_debug and _HAS_CV2

        fb = min(self.front_band_deg, self.angle_range_deg)

        if draw:
            lidar = np.full((self.IMG_H, self.IMG_W, 3), 255, dtype=np.uint8)

            # 전방(위쪽) 화살표
            roi_px = int(self.roi_distance * 100)
            cv2.arrowedLine(
                lidar, (self.cx, self.cy), (self.cx, self.cy - roi_px),
                (0, 128, 128), 1, tipLength=0.1
            )

            # ROI 부채꼴
            overlay = lidar.copy()
            start_ang = 270.0 - self.angle_range_deg
            end_ang   = 270.0 + self.angle_range_deg
            cv2.ellipse(
                overlay, (self.cx, self.cy), (roi_px, roi_px),
                0, start_ang, end_ang, (0, 255, 255), -1
            )
            lidar = cv2.addWeighted(overlay, 0.3, lidar, 0.7, 0)

            # 좌/우 경계선
            rad_L = math.radians(+self.angle_range_deg)
            rad_R = math.radians(-self.angle_range_deg)
            end_L = (int(self.cx + roi_px * math.sin(rad_L)),
                     int(self.cy - roi_px * math.cos(rad_L)))
            end_R = (int(self.cx + roi_px * math.sin(rad_R)),
                     int(self.cy - roi_px * math.cos(rad_R)))
            cv2.line(lidar, (self.cx, self.cy), end_L, (200, 0, 200), 1)
            cv2.line(lidar, (self.cx, self.cy), end_R, (200, 0, 200), 1)

            # 전방 밴드 경계선(±front_band_deg)
            rad_FL = math.radians(+fb)
            rad_FR = math.radians(-fb)
            end_FL = (int(self.cx + roi_px * math.sin(rad_FL)),
                      int(self.cy - roi_px * math.cos(rad_FL)))
            end_FR = (int(self.cx + roi_px * math.sin(rad_FR)),
                      int(self.cy - roi_px * math.cos(rad_FR)))
            cv2.line(lidar, (self.cx, self.cy), end_FL, (150, 150, 150), 1)
            cv2.line(lidar, (self.cx, self.cy), end_FR, (150, 150, 150), 1)

        # --- 버킷 ---
        flag_R = flag_L = flag_F = False
        distance_R, angle_R, pts_R = [], [], []
        distance_L, angle_L, pts_L = [], [], []
        distance_F, angle_F, pts_F = [], [], []

        count = len(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        for i in range(count):
            rng = scan.ranges[i]
            if not math.isfinite(rng) or rng <= 0.0:
                continue
            
            degree_raw = math.degrees(angle_min + angle_inc * i)
            degree = self._wrap180(degree_raw + self.front_offset_deg)  # +는 CCW(반시계) 회전
            rad = math.radians(degree)

            # 정면 기준 각도 [-180, +180)
            degree_raw = math.degrees(angle_min + angle_inc * i)
            degree = ((degree_raw - self.front_offset_deg + 180.0) % 360.0) - 180.0
            rad = math.radians(degree)

            # 시각화 좌표 (정면=위쪽), 1 m ~= 100 px (3 m를 half-canvas로)
            px = int(self.cx + ((rng / 3.0) * math.sin(rad) * self.cx))
            py = int(self.cy - ((rng / 3.0) * math.cos(rad) * self.cy))

            # ROI 범위
            in_angle = (-self.angle_range_deg <= degree <= self.angle_range_deg)
            in_dist_for_use = (self.hard_stop_dist <= rng < self.roi_distance)
            in_front_band = (abs(degree) <= fb)

            # 점 시각화
            if draw:
                if in_angle and in_dist_for_use:
                    if in_front_band:
                        cv2.circle(lidar, (px, py), 1, (0, 165, 255), -1)  # 오렌지: 전방밴드 사용점
                    else:
                        cv2.circle(lidar, (px, py), 1, (0, 0, 255), -1)    # 빨강: L/R 사용점
                elif in_angle:
                    cv2.circle(lidar, (px, py), 1, (200, 200, 200), -1)     # 연회색: 각도만 ROI
                else:
                    cv2.circle(lidar, (px, py), 1, (255, 0, 0), -1)         # 파랑: ROI 밖

            # 버킷 분류
            if in_angle and in_dist_for_use:
                if in_front_band:
                    distance_F.append(rng); angle_F.append(degree); pts_F.append((px, py)); flag_F = True
                elif degree < 0.0:
                    distance_R.append(rng); angle_R.append(degree); pts_R.append((px, py)); flag_R = True
                else:
                    distance_L.append(rng); angle_L.append(degree); pts_L.append((px, py)); flag_L = True

        # 좌/우/전방 최소 거리
        min_idx_R = min_idx_L = min_idx_F = 0
        self.min_distance_R = float('inf')
        self.min_distance_L = float('inf')
        self.min_distance_F = float('inf')

        for i, d in enumerate(distance_L):
            if d < self.min_distance_L:
                self.min_distance_L = d; min_idx_L = i
        for i, d in enumerate(distance_R):
            if d < self.min_distance_R:
                self.min_distance_R = d; min_idx_R = i
        for i, d in enumerate(distance_F):
            if d < self.min_distance_F:
                self.min_distance_F = d; min_idx_F = i

        # 좌/우 비대칭 오차(err) 계산 
        err = 0.0
        if flag_L and flag_R and len(pts_L) > 0 and len(pts_R) > 0:
            if draw:
                cv2.arrowedLine(lidar, (self.cx, self.cy), pts_L[min_idx_L], (0, 255, 0), 1, tipLength=0.1)
                cv2.arrowedLine(lidar, (self.cx, self.cy), pts_R[min_idx_R], (255, 0, 0), 1, tipLength=0.1)
            angle_sum = angle_R[min_idx_R] + angle_L[min_idx_L]
            if angle_sum != 0.0:
                err = (
                    (self.roi_distance * 100 - pts_L[min_idx_L][0]) -
                    (pts_R[min_idx_R][0] - (self.roi_distance * 100 + self.cx))
                ) / 2.0
            else:
                err = (
                    (self.cx + self.roi_distance * 100) - pts_R[min_idx_R][0] -
                    (pts_L[min_idx_L][0] - (self.cx - self.roi_distance * 100))
                ) / 2.0

        elif flag_R and not flag_L and len(pts_R) > 0:
            if draw:
                # 가상 왼쪽 경계 참조 화살표
                cv2.arrowedLine(lidar, (self.cx, self.cy), end_L, (255, 0, 255), 1, tipLength=0.1)
                if angle_R[min_idx_R] != 0.0:
                    cv2.arrowedLine(lidar, (self.cx, self.cy), pts_R[min_idx_R], (255, 0, 0), 1, tipLength=0.1)
            err = (self.cx + self.roi_distance * 100) - pts_R[min_idx_R][0]

        elif flag_L and not flag_R and len(pts_L) > 0:
            if draw:
                # 가상 오른쪽 경계 참조 화살표
                cv2.arrowedLine(lidar, (self.cx, self.cy), end_R, (153, 0, 153), 1, tipLength=0.1)
                cv2.arrowedLine(lidar, (self.cx, self.cy), pts_L[min_idx_L], (0, 255, 0), 1, tipLength=0.1)
            err = -(pts_L[min_idx_L][0] - (self.cx - self.roi_distance * 100))

        else:
            if draw:
                cv2.line(lidar, (self.cx, self.cy), (self.cx, self.cy - roi_px), (180, 180, 180), 1)
            err = 0.0

        # 정규화 & 평활화
        denom = max(1.0, self.roi_distance * 100.0)
        err_norm = float(np.clip(err / denom, -1.0, 1.0))
        self.err_f = (1.0 - self.error_alpha) * self.err_f + self.error_alpha * err_norm
        self.error = err

        # 디버그 텍스트
        if draw:
            try:
                def fmt(v): return "inf" if not math.isfinite(v) else f"{v:.2f}m"
                txt1 = f"err_norm={self.err_f:+.2f}  dL={fmt(self.min_distance_L)}  dR={fmt(self.min_distance_R)}  dF={fmt(self.min_distance_F)}"
                cv2.putText(lidar, txt1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
                mode_en = self.mode.name
                cv2.putText(lidar, f"mode: {mode_en}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                cv2.imshow('lidar', lidar)
                cv2.waitKey(1)
            except Exception:
                pass


    # 상태 결정 
    def decide_mode(self, d_near: float, err_f: float) -> Mode:
        # 하드스톱 
        if d_near < self.hard_stop_dist:
            return Mode.HARD_STOP

        # 제로턴 구간 (히스테리시스: 진입/이탈 분리)
        if self.mode == Mode.ZERO_TURN:
            if d_near < self.zero_turn_exit:
                return Mode.ZERO_TURN
        else:
            if d_near < self.zero_turn_dist:
                return Mode.ZERO_TURN

        # 좌/우 회전 진입/유지 (히스테리시스)
        ae = abs(err_f)
        if self.mode in (Mode.TURN_LEFT, Mode.TURN_RIGHT):
            if ae > self.turn_exit_err:  # 아직 크면 계속 회전 유지
                return self.mode
            else:
                return Mode.STRAIGHT
        else:
            if ae >= self.turn_entry_err:
                return Mode.TURN_LEFT if err_f > 0.0 else Mode.TURN_RIGHT

        # 그 외는 직진
        return Mode.STRAIGHT

    # ---------------------------
    # 주기적으로 /cmd_vel 발행
    # ---------------------------
    def publish_cmd(self):
        dL = self.min_distance_L
        dR = self.min_distance_R
        dF = self.min_distance_F
        d_near = min(dL, dR, dF)  # 전방 포함
        err_f = self.err_f

        # 1) 상태 결정
        new_mode = self.decide_mode(d_near, err_f)
        if new_mode != self.mode:
            self._last_mode = self.mode
            self.mode = new_mode
            self._mode_changed_ts = time.time()
            if self.mode == Mode.TURN_LEFT:
                self.get_logger().info("현재 좌회전중 (mode=TURN_LEFT)")
            elif self.mode == Mode.TURN_RIGHT:
                self.get_logger().info("현재 우회전중 (mode=TURN_RIGHT)")
            elif self.mode == Mode.ZERO_TURN:
                self.get_logger().info("현재 제로턴중 (mode=ZERO_TURN)")
            elif self.mode == Mode.STRAIGHT:
                self.get_logger().info("현재 직진중 (mode=STRAIGHT)")
            elif self.mode == Mode.HARD_STOP:
                self.get_logger().warn("안전 정지 (mode=HARD_STOP)")

        # 2) 상태 → 명령 변환
        v_cmd = 0.0
        w_cmd = 0.0

        if self.mode == Mode.HARD_STOP:
            v_cmd = 0.0
            w_cmd = 0.0

        elif self.mode == Mode.ZERO_TURN:
            v_cmd = 0.0
            # err가 0이면 가까운 쪽을 피하는 방향으로 회전
            if err_f == 0.0:
                if math.isfinite(dL) or math.isfinite(dR):
                    turn_sign = +1.0 if dR < dL else -1.0
                else:
                    turn_sign = +1.0  # 둘 다 inf면 좌회전 기본값
            else:
                turn_sign = +1.0 if err_f > 0.0 else -1.0
            w_mag = max(self.w_zero_min, abs(self.kp_ang_zero * err_f))
            w_cmd = float(np.clip(turn_sign * w_mag, -self.w_zero_max, self.w_zero_max))

        elif self.mode == Mode.TURN_LEFT or self.mode == Mode.TURN_RIGHT:
            v_cmd = self.v_turn
            turn_sign = +1.0 if self.mode == Mode.TURN_LEFT else -1.0
            w_mag = max(self.w_turn_min, abs(self.kp_ang * err_f))
            w_cmd = float(np.clip(turn_sign * w_mag, -self.w_max, self.w_max))

        elif self.mode == Mode.STRAIGHT:
            v_cmd = self.v_straight
            w_cmd = 0.0  # 진짜 직진

        # 방향 반전 옵션
        if self.invert_angular:
            w_cmd = -w_cmd

        # 전방 가까우면 직진/회전 중에도 감속 (제로턴/하드스톱 제외)
        if self.mode not in (Mode.ZERO_TURN, Mode.HARD_STOP):
            if math.isfinite(dF) and dF < self.slowdown_distance:
                # hard_stop_dist ~ slowdown_distance 구간을 [0..1]로 매핑
                alpha = (dF - self.hard_stop_dist) / max(1e-6, (self.slowdown_distance - self.hard_stop_dist))
                alpha = float(np.clip(alpha, 0.0, 1.0))
                v_cmd = max(self.v_slow, alpha * v_cmd)

        # STM32 스케일 보정
        v_cmd *= self.SCALE_V
        w_cmd *= self.SCALE_W

        # 3) 발행
        msg = Twist()
        msg.linear.x  = float(v_cmd)
        msg.linear.y  = 0.0
        msg.linear.z  = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

        # 4) 주기 로그
        now = time.time()
        if (now - self._last_log) >= self.log_period_s:
            self._last_log = now
            self.get_logger().info(
                f"/cmd_vel: v={msg.linear.x:+.3f} m/s, w={msg.angular.z:+.3f} rad/s | "
                f"mode={self.mode.name}, err={err_f:+.2f}, dL={dL:.2f}m, dR={dR:.2f}m, dF={dF:.2f}m"
            )


def main():
    rclpy.init()
    node = AvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if _HAS_CV2:
                cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
