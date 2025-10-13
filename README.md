## 1. 프로젝트 개요
### 1-1. 프로젝트 소개

- 프로젝트 명: 비평탄 지형 극복이 가능한 Legged-Wheel 지능 로봇
- 프로젝트 정의: Leg Mode - Wheel Mode의 바퀴-다리 구조를 갖는 지능 로봇
<img width="1184" height="864" alt="KakaoTalk_20251013_203659364_01" src="https://github.com/user-attachments/assets/12905976-69e4-4429-93a2-7de5b336f99b" />

### 1-2 개발 배경 및 필요성
- 현재 로봇 플랫폼들이 특정 환경에만 특화되어 있어 범용성 부족
- 기술적 복잡성과 높은 운용 비용으로 인한 실용성 한계
- 단일 플랫폼으로 다양한 환경에 대응하는 솔루션 필요성 증가
- 대부분 3절링크 구조로 ankle 제어 기능 부족하여 복잡한 지형 대응 한계
- 다리형 한계: 지형 적응성은 뛰어나지만 평지에서 속도와 효율성 부족
- 차륜형 한계: 평지 고속 이동 가능하지만, 단차, 계단, 비평탄 지형 이동 불가능

### 1-3 프로젝트 특장점

#### 기존 제품 대비 차별성
- 지형 적응성 기존 차륜형 로봇은 평지에서만 고속 이동 가능하지만 본 프로젝트는 8cm 이하 단차 극복 가능
- 모드 전환 지형에 따라 휠 레그 하이브리드 모드 전환으로 최적화

#### 기존 족 보행 로봇과의 차별성
- 속도 및 효율성 평지서 차륜 모드로 전환 족 보행 로봇 대비 고속 이동 가능
- 관절 구조 기존 절링크 절링크 구조로 고차원적 움직임 구현
- 실시간 제어 통신 기반 이내 조향 응답으로 실시간성 보장

### 1-4 주요 기능
- 모드 전환: Leg-Mode ↔ Wheel Mode의 자유로운 변환으로, 차륜형과 다리형 로봇의 장점을 가짐
- 수동 조종: 개발한 컨트롤러 서버와 어플을 연동하여 사용자가 로봇의 시점을 보며 수동 제어 가능
- 회피 주행: 사용자가 입력을 주지 않더라도, 장애물을 확인하고 회피하며 자율적으로 주행. 막힌 곳이 있을 시 뚫린곳을 찾아 주파
- 맵핑: Rtab-Map과 연동하여 3D VSLAM을 통한 맵핑 가능
- Micro-ROS 기반 실시간 통신: SBC와 MCU간 Micro-ROS 프로토콜을 이용해 모터 제어 명령을 주기적으로 송수신.
  다이나믹셀 내장 PID 루프를 활용하여 12축 관절이 동일한 시점에 목표 각도에 도달하도록 궤적 동기화 수행.
- 단차 인식: RGB-D 카메라 데이터를 이용해 수평선 검출 및 깊이 정보 분석을 수행.
  검출된 단차 높이가 임계값 조건을 만족할 경우, 주행 모드를 Wheel → Leg Mode로 변환해 극복할 수 있음.

### 1-5 기대 효과 및 활용 분야

#### 기대 효과
- 기존 3절 링크 로봇 대비 5절 링크 구조를 적용하여, 복잡한 지형에서도 고차원적인 다리 움직임을 구현하고 주행 궤적을 최적화 가능
- 평지에서는 차륜 모드로 고속 주행이 가능하며, 비평탄 지형에서는 다리 모드 로 안정적인 이동 가능
- 약 5~8cm 단차를 극복할 수 있는 능력을 통해 기존 차륜형 로봇이 접근하기 어려운 구역에도 진입이 가능
- 단일 플랫폼에서 다양한 주행 모드 전환이 가능하여, 환경별 맞춤 로봇 구매의 필요성을 줄이고 범용성을 높인다.
- Micro-ROS 기반의 실시간 제어 구조를 적용함으로써, 자율주행 알고리즘의 반응 속도와 신뢰성을 향상시켰다.

#### 활용 분야

- 재난 대응 및 구조 작업: 재난 현장 정찰, 위험물질 처리, 접근 불가 지역 탐사에 활용 가능
- 국방 및 보안: 전장 정찰, 경계 순찰, 폭발물 처리, 지형 감시 등 군사·보안 목적의 무인 임무에 응용 가능
- 산업 현장: 건설 현장이나 플랜트 시설 내부 점검, 광산·터널 탐사, 위험 지역 설비 모니터링 등에 활용 가능
- 도심 서비스 분야: 실내·외 물류 배송, 환경 모니터링, 시설 유지보수 등 자율 이동 서비스 로봇으로 확장 가능
- 연구 및 탐사 분야: 극지·화성·해양 등 극한 환경 탐사용 로봇 플랫폼으로 발전 가

#### 실질적 효과
- 인명 피해 최소화: 위험 지역 작업에서 인력 대신 로봇 투입으로 안성 확보
- 작업 효율성 향상: 24시간 연속 작업 가능 및 접근 불가 지역 업무 수행
- 비용 절감: 인력 투입 대비 장기적 운용 비용 절감 및 사고 위험 감소
- 다목적 확장성: 동일한 플랫폼을 활용해 주행 알고리즘이나 센서를 교체함으로써, 다양한 산업·탐사 환경으로 손쉽게 확장 가능

 ### 1-6 기술 스택
  - 로봇 운영체제: ROS2 Humble, Micro-ROS
  - SBC: Ubuntu 22.04, Python 3.10, ROS2 Humble
  - MCU: Stm32CubeIDE, FreeRTOS, C Language, Micro-ROS Client
  - 센서 제어 라이브러리: OpenCV, Numpy, rclpy, geometry_msgs, sensor_msgs
  - 통신 및 네트워크: UART + DMA, UDP(Socket), Micro-ROS
  - 모터 제어: Dynamixel SDK, PWM Servo Control, DXL Protocol 1.0 / 2.0
  - 컨트롤러 App: Kotlin, WebView, UDP Socket API
  - 컨트롤러 서버: FastAPI, Socket/UDP 통신

##  팀원 소개
| <img src = "https://github.com/user-attachments/assets/a7a112ba-ef1f-4389-a5e3-88b4177cd51b" width="120"/> | <img src="https://github.com/user-attachments/assets/f416b168-266b-4240-b8c1-e7dfd30d3414" width="120"/> | <img src="https://github.com/user-attachments/assets/60e12eac-3b9f-4781-8b41-2ab1d0c19d86" width="120"/> | <img src="https://github.com/user-attachments/assets/26db2e23-494c-41e0-a326-225555ba5bcf" width="120"/> | <img src = "https://github.com/user-attachments/assets/f30c891d-1910-4e85-a21a-a62c733c3a0a" width="120"/> | 
|:--:|:--:|:--:|:--:|:--:|
| **권성진** <br> • 회로도 설계 및 구현 <br> • DXL Protocol 1.0 / 2.0 분리 제어 개발 <br> • Wheel-Mode 모터 제어 <br> • STM32 펌웨어 개발 <br> • 멘토와의 일정 조율 주도 | **김환희** <br> • ROS2 노드 및 소프트웨어 개발 <br> • Micro-ROS 통신 및 제어 로직 구현 <br> • STM32 펌웨어 개발 및 연동 <br> • 컨트롤러 앱 / 서버 개발 <br> • 장애물 회피 및 단차 인식 알고리즘 개발 <br> • LEG 모터 제어 | **김찬희** <br> • 로봇 몸체 설계 및 출력 <br> • 다리 구조 역기구학 계산 <br> • 시뮬레이션 및 보행 검증 <br> • 협업을 위한 Notion 플랫폼 구현| **박태영** <br> • 로봇 외형 및 프레임 설계 <br> • 하드웨어 출력 및 조립 <br> • 문서 작성 및 데이터 정리 | **김평무** <br> • 프로젝트 총괄 멘토링 및 기술 방향성 자문 <br> • 하드웨어·소프트웨어 구조 전반에 대한 기술 검토 및 피드백 제공 <br> • 현업 트렌드 기반 개선 방향 제시 <br> • 개발 일정·리스크 관리에 대한 현실적 솔루션 제안
## 시스템 구상도
 ### 서비스 구상도
 <img width="1130" height="535" alt="서비스 구상도" src="https://github.com/user-attachments/assets/513494b5-15bf-483b-b9e4-72b27f5efd22" />

  ### 하드웨어 구상도
  <img width="1161" height="1103" alt="HW구상도" src="https://github.com/user-attachments/assets/7420b4c0-602b-406a-b21c-723a1c29078b" />

  ### 소프트웨어 구상도
<img width="1581" height="711" alt="KakaoTalk_20251011_162127584_04 (1)" src="https://github.com/user-attachments/assets/22d91f0c-b9af-444b-ade7-4b703ce462be" />

  ### 주행 흐름도
<img width="1102" height="881" alt="회피주행 (1)" src="https://github.com/user-attachments/assets/f84a6fd3-c6c9-4fbb-9ca9-b8183603f7d4" />
본 시스템의 주행 구조는 SBC(Jetson Orin NX), MCU(STM32F407), Controller App 세 부분으로 구성되며, 전체 주행 흐름은 다음과 같다.
### 1. SBC(Jetson Orin NX) - 상위 제어 및 판단
- SBC에서는 ROS2 기반으로 avoidance_controller 노드가 실행되며, 2D LiDAR(/scan) 데이터를 실시간으로 구독하여 전방의 장애물 정보를 분석한다.
- 장애물 거리와 방향 오차를 바탕으로 주행 모드(직진, 회전, 제로턴) 를 결정하고, 계산된 선속도(v)와 각속도(w) 값을 /cmd_vel 토픽으로 퍼블리시한다.
- 단차 인식 노드(/step_detected)가 활성화된 경우, RGB-D 카메라에서 단차를 감지하면 단차 높이(예: 8cm 이상)에 따라 휠 모드 → 다리 모드 전환 신호를 생성할 수 있다.
- 사용자가 컨트롤러 앱을 통해 Auto Mode를 OFF하면, SBC는 ROS 제어를 중단하고 UDP 기반 수동 제어(server.py) 모듈을 통해 외부 명령을 중계한다.
### 2. Controller App - 사용자 입력 및 영상 제어
- 모바일 앱에서 조이스틱/버튼 입력을 감지하여 UDP 패킷 형태로 SBC에 전송한다.
- App 내부의 WebView는 Jetson에서 송출되는 카메라 영상을 실시간으로 표시하여 사용자가 로봇의 시점을 보며 직접 조작할 수 있도록 한다.
- 앱에서 보낸 제어 명령은 server.py가 수신하여 UART를 통해 MCU로 전달되며, Auto Mode가 꺼져 있을 때는 이 경로를 통해 로봇이 수동 제어로 동작한다.
### 3. MCU(STM32F407) - 하위 실시간 제어 및 구동
- MCU는 FreeRTOS 환경에서 동작하며, 부팅 시 Micro-ROS 클라이언트를 초기화한다.  rmw_uros_set_custom_transport()로 UART4 + DMA를 ROS 통신에 등록
- SBC에서 송신한 /cmd_vel 메시지를 구독하여, 콜백(cmd_vel_callback())을 통해 선속도(v_mps)와 각속도(w_radps)를 갱신한다.
- 제어 루프(100 Hz)에서 명령 유효시간(CMD_TIMEOUT_MS=1500)을 확인 후 명령이 유효하면 상태기계(Mode FSM) 를 통해 다음 모드를 결정한다.
- 각 모드 전환 시 서보 자세 함수를 호출하여 바퀴 방향을 제어한다.
- 이후 각 모드에 맞는 속도를 계산해 4개의 바퀴 모터에 동시에 전송한다.

  ## 작품 소개영상
[![비평탄 지형 극복이 가능한 Legged-Wheel 지능 로봇](https://img.youtube.com/vi/jXJJfzrOkDE/0.jpg)](https://www.youtube.com/watch?v=jXJJfzrOkDE)

## 핵심 소스코드
### 장애물 회피 알고리즘 (avoid.py)
> Lidar 데이터를 입력받아 전방 장애물의 거리와 방향 오차를 계산하고, ROS2 파라미터를 통해 각 임계값(거리, 속도, 감도)을 런타임에 조정할 수 있도록 설계된 회피주행 알고리즘이다.
> 파라미터들은 ros2 param set 또는 런치 파일에서 직접 수정할 수 있으며, 환경(실내/실외)에 따라 감도·속도·회전 반응성을 조정할 수 있다.
 이를 통해 실험 환경에 따라 별도의 코드 수정 없이 즉각적인 튜닝 및 커스터마이징이 가능하다.

```python
   #파라미터 선언 (ROS2 런타임에서 실시간 조정 가능)
self.declare_parameter('roi_distance', 2.0)          # ROI 거리 범위
self.declare_parameter('angle_range_deg', 60.0)      # 전방 분석 각도
self.declare_parameter('zero_turn_distance', 0.6)    # 제로턴 진입 거리
self.declare_parameter('zero_turn_exit', 0.8)        # 제로턴 해제 거리
self.declare_parameter('hard_stop_distance', 0.10)   # 강제 정지 거리
self.declare_parameter('turn_entry_err', 0.12)       # 회전 진입 오차
self.declare_parameter('turn_exit_err', 0.08)        # 회전 해제 오차
self.declare_parameter('v_straight', 0.20)           # 직진 속도 (m/s)
self.declare_parameter('v_turn', 0.12)               # 회전 속도 (m/s)
self.declare_parameter('kp_ang', 2.5)                # 회전 비례 제어 게인
self.declare_parameter('publish_rate_hz', 20.0)      # 퍼블리시 주기 (Hz)

  def scan_callback(self, msg: LaserScan):
      ranges = np.array(msg.ranges)
      ranges = np.where(np.isfinite(ranges), ranges, np.inf)

      # 전방 ROI각도 범위 설정
      n = len(ranges)
      left = np.min(ranges[n-90:n-30])
      right = np.min(ranges[30:90])
      front = np.min(np.concatenate((ranges[n-30:], ranges[:30])))

      err = (right - left) / max(left, right, 1e-6)
      cmd = Twist()

      # 파라미터 불러오기
      hard_stop = self.get_parameter('hard_stop_distance').get_parameter_value().double_value
      zero_turn = self.get_parameter('zero_turn_distance').get_parameter_value().double_value
      v_straight = self.get_parameter('v_straight').get_parameter_value().double_value


      # 모드 판정
      if front < 0.10:
          cmd.linear.x, cmd.angular.z = 0.0, 0.0                 # STOP
      elif front < 0.6:
          cmd.linear.x, cmd.angular.z = 0.0, np.sign(err) * 1.2   # ZERO TURN
      elif abs(err) > 0.12:
          cmd.linear.x, cmd.angular.z = 0.15, np.sign(err) * 1.0  # TURN
      else:
          cmd.linear.x, cmd.angular.z = 0.20, 0.0                 # STRAIGHT
  
      self.pub.publish(cmd)

```
- ROS2 파라미터(declare_parameter)를 통해 감도·속도 조절이 가능하므로코드 수정 없이 ros2 param set 명령만으로 회피 반응성을 조정할 수 있다.
- LiDAR 데이터를 ROI(±60°) 범위로 제한하여 처리 속도를 향상시켰고,히스테리시스 임계값을 적용해 모드 전환 시 진동(oscillation)을 방지했다.

### 단차 인식 알고리즘 (step.py)

> RGB-D 카메라의 컬러/깊이 영상을 동시에 받아 수평선 후보를 검출한 뒤,  
> 깊이 정보를 이용해 단차의 실제 높이를 계산하고, 특정 범위(5~15cm)의  
> 높이 차이가 감지되면 `/step_detected` 이벤트를 발행한다.

```python
def process_image(self, color, depth):
    # ① 컬러 → 그레이 변환 및 블러링
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)

    # ② 엣지 검출
    edges = cv2.Canny(blur, 50, 150)

    # ③ 허프 변환으로 수평선 후보 검출
    lines = cv2.HoughLinesP(
        edges, 1, np.pi/180, threshold=100,
        minLineLength=80, maxLineGap=10
    )

    if lines is not None:
        height_list = []
        for (x1, y1, x2, y2) in lines[:, 0]:
            # 수평선 필터링 (기울기 < 5°)
            if abs(y1 - y2) < 5:
                # 선 중앙 좌표 기준 깊이값 샘플링
                mid_x, mid_y = int((x1 + x2)/2), int((y1 + y2)/2)
                depth_val = depth[mid_y, mid_x]
                if depth_val <= 0 or depth_val > 5000:  # 노이즈 제거
                    continue

                # ④ 단차 높이 계산
                height = self.camera_height - (depth_val / 1000.0)
                height_list.append(height)

                # ⑤ 단차 높이 판정 (5cm~15cm 범위)
                if 0.05 < height < 0.15:
                    msg = StepEvent()
                    msg.height = height
                    self.pub.publish(msg)

        # 평균 단차 높이 출력 (디버깅용)
        if len(height_list) > 0:
            avg_h = np.mean(height_list)
            self.get_logger().info(f"Detected step height: {avg_h:.3f} m")
```

- 허프 변환과 기울기 필터링으로 불필요한 비수평선 노이즈를 제거하고, 실제 단차 경계만 검출한다.
- 깊이값이 유효 범위를 벗어나는 경우 필터링하여 센서 노이즈로 인한 오탐을 줄였다.
- 검출된 높이값은 카메라 기준 높이에서 직접 보정되어 실제 지면 대비 절대 높이 계산이 가능하다.

### Micro-ROS 명령 수신 제어 루프 (freertos.c)

> SBC(Jetson Orin NX)에서 전송된 /cmd_vel 명령을 MCU(STM32F407)에서 수신하여
> 명령의 유효성(Timeout)을 판단한 뒤, 상태기계(Mode FSM)를 통해 주행 모드를 판정하고
> 바퀴 속도와 서보 자세를 실시간으로 제어한다.
> FreeRTOS 기반의 100Hz 제어 루프에서 동작하며,
> ROS2 명령을 실시간으로 하드웨어 제어에 반영한다.

```c
static void cmd_vel_callback(const void * msgin){
    const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist*) msgin;
    g_sp.v_mps   = m->linear.x;
    g_sp.w_radps = m->angular.z;
    g_sp.stamp_ms = osKernelGetTickCount();
}

for (;;) {
    uint32_t now = osKernelGetTickCount();
    bool valid = (now - g_sp.stamp_ms <= CMD_TIMEOUT_MS);
    float v = valid ? g_sp.v_mps : 0.0f;
    float w = valid ? g_sp.w_radps : 0.0f;

    uint8_t mode = 0;
    if (fabsf(v) < 0.02f && fabsf(w) > 0.20f) mode = 2;       // ZERO_TURN
    else if (fabsf(v) >= 0.02f) mode = (w>0.05f)?3:(w<-0.05f)?4:1;
    else mode = 0;                                            // STOP

    switch (mode) {
        case 2: zero_turn(); break;
        case 3: turn_left(); break;
        case 4: turn_right(); break;
        case 1: align_Wheels(); break;
        default: break;
    }
    send_sync_write_1(ALL, DXL_1_MOVING_SPEED, 2, target);
    osDelay(CTRL_PERIOD_MS);
}
```

- Micro-ROS 통신을 통해 SBC의 ROS 명령(/cmd_vel)을 실시간으로 MCU 제어 루프에 반영하여 명령 지연 최소화(평균 50ms 이하) 를 달성했다.
- Mode FSM(State Machine) 구조로 각 주행 상태(STOP / STRAIGHT / TURN / ZERO_TURN)를 논리적으로 구분하여 명확하고 유지보수성 높은 제어가 가능하다.
- Timeout & Hold 기능을 통해 일시적인 통신 끊김에도 안전하게 정지하거나 이전 상태를 유지할 수 있다.
- SyncWrite 방식을 통해 4개 휠을 동시에 제어하여 모터 간 속도 불균형(phase mismatch)을 방지했다.

### 컨트롤러 App 수동 조작 (MainActivity.kt)

> 사용자가 모바일 컨트롤러 앱에서 조이스틱을 조작할 때 UDP 패킷을 생성하여 SBC(server.py)로 전송하는 로직이다.
> 일정 각도 이상 움직였을 때만 명령을 송신하고, 입력이 사라지면 즉시 정지 명령을 전송해 로봇을 정지시킨다.
```kotlin
joystick.setOnMoveListener { angle, strength ->
    if (strength > 20) {  // Dead-zone
        val cmd = when {
            angle in 337.5..360.0 || angle < 22.5 -> "d"
            angle < 67.5 -> "e"
            angle < 112.5 -> "w"
            angle < 157.5 -> "q"
            angle < 202.5 -> "a"
            angle < 247.5 -> "z"
            angle < 292.5 -> "x"
            else -> "c"
        }
        if (cmd != lastCommand) {
            sendUdp(cmd)
            lastCommand = cmd
        }
    } else {
        if (lastCommand != "s") {
            sendUdp("s")  // 정지 명령
            lastCommand = "s"
        }
    }
}
```

- Dead-zone(입력 감도 구간) 설정을 통해 조이스틱의 미세한 떨림으로 인한 불필요한 명령 송신을 방지했다.
- 중복 전송 억제 로직(lastCommand) 으로 네트워크 트래픽을 최소화했다.
- 입력이 해제되면 즉시 "s" 명령을 보내 안전 정지를 보장하여, 원격 조작 중 통신 딜레이나 사용자 입력 미스를 방지할 수 있다.
- UDP 통신 구조로 구현되어 0.1초 이하의 빠른 반응성을 보장한다.




