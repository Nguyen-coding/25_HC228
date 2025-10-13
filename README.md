## 1. 프로젝트 개요
1-1. 프로젝트 소개

- 프로젝트 명: 비평탄 지형 극복이 가능한 Legged-Wheel 지능 로봇
- 프로젝트 정의: Leg Mode - Wheel Mode의 바퀴-다리 구조를 갖는 지능 로봇
<img width="1184" height="864" alt="KakaoTalk_20251013_203659364_01" src="https://github.com/user-attachments/assets/12905976-69e4-4429-93a2-7de5b336f99b" />

1-2 개발 배경 및 필요성
- 현재 로봇 플랫폼들이 특정 환경에만 특화되어 있어 범용성 부족
- 기술적 복잡성과 높은 운용 비용으로 인한 실용성 한계
- 단일 플랫폼으로 다양한 환경에 대응하는 솔루션 필요성 증가
- 대부분 3절링크 구조로 ankle 제어 기능 부족하여 복잡한 지형 대응 한계
- 다리형 한계: 지형 적응성은 뛰어나지만 평지에서 속도와 효율성 부족
- 차륜형 한계: 평지 고속 이동 가능하지만, 단차, 계단, 비평탄 지형 이동 불가능

1-3 프로젝트 특장점

기존 제품 대비 차별성
- 지형 적응성 기존 차륜형 로봇은 평지에서만 고속 이동 가능하지만 본 프로젝트는 8cm 이하 단차 극복 가능
- 모드 전환 지형에 따라 휠 레그 하이브리드 모드 전환으로 최적화

기존 족 보행 로봇과의 차별성
- 속도 및 효율성 평지서 차륜 모드로 전환 족 보행 로봇 대비 고속 이동 가능
- 관절 구조 기존 절링크 절링크 구조로 고차원적 움직임 구현
- 실시간 제어 통신 기반 이내 조향 응답으로 실시간성 보장

1-4 주요 기능
- 모드 전환: Leg-Mode ↔ Wheel Mode의 자유로운 변환으로, 차륜형과 다리형 로봇의 장점을 가짐
- 수동 조종: 개발한 컨트롤러 서버와 어플을 연동하여 사용자가 로봇의 시점을 보며 수동 제어 가능
- 회피 주행: 사용자가 입력을 주지 않더라도, 장애물을 확인하고 회피하며 자율적으로 주행. 막힌 곳이 있을 시 뚫린곳을 찾아 주파
- 맵핑: Rtab-Map과 연동하여 3D VSLAM을 통한 맵핑 가능
- Micro-ROS 기반 실시간 통신: SBC와 MCU간 Micro-ROS 프로토콜을 이용해 모터 제어 명령을 주기적으로 송수신.
  다이나믹셀 내장 PID 루프를 활용하여 12축 관절이 동일한 시점에 목표 각도에 도달하도록 궤적 동기화 수행.
- 단차 인식: RGB-D 카메라 데이터를 이용해 수평선 검출 및 깊이 정보 분석을 수행.
  검출된 단차 높이가 임계값 조건을 만족할 경우, 주행 모드를 Wheel → Leg Mode로 변환해 극복할 수 있음.

1-5 기대 효과 및 활용 분야

기대 효과
- 기존 3절 링크 로봇 대비 5절 링크 구조를 적용하여, 복잡한 지형에서도 고차원적인 다리 움직임을 구현하고 주행 궤적을 최적화 가능
- 평지에서는 차륜 모드로 고속 주행이 가능하며, 비평탄 지형에서는 다리 모드 로 안정적인 이동 가능
- 약 5~8cm 단차를 극복할 수 있는 능력을 통해 기존 차륜형 로봇이 접근하기 어려운 구역에도 진입이 가능
- 단일 플랫폼에서 다양한 주행 모드 전환이 가능하여, 환경별 맞춤 로봇 구매의 필요성을 줄이고 범용성을 높인다.
- Micro-ROS 기반의 실시간 제어 구조를 적용함으로써, 자율주행 알고리즘의 반응 속도와 신뢰성을 향상시켰다.

활용 분야

- 재난 대응 및 구조 작업: 재난 현장 정찰, 위험물질 처리, 접근 불가 지역 탐사에 활용 가능
- 국방 및 보안: 전장 정찰, 경계 순찰, 폭발물 처리, 지형 감시 등 군사·보안 목적의 무인 임무에 응용 가능
- 산업 현장: 건설 현장이나 플랜트 시설 내부 점검, 광산·터널 탐사, 위험 지역 설비 모니터링 등에 활용 가능
- 도심 서비스 분야: 실내·외 물류 배송, 환경 모니터링, 시설 유지보수 등 자율 이동 서비스 로봇으로 확장 가능
- 연구 및 탐사 분야: 극지·화성·해양 등 극한 환경 탐사용 로봇 플랫폼으로 발전 가

실질적 효과
- 인명 피해 최소화: 위험 지역 작업에서 인력 대신 로봇 투입으로 안성 확보
- 작업 효율성 향상: 24시간 연속 작업 가능 및 접근 불가 지역 업무 수행
- 비용 절감: 인력 투입 대비 장기적 운용 비용 절감 및 사고 위험 감소
- 다목적 확장성: 동일한 플랫폼을 활용해 주행 알고리즘이나 센서를 교체함으로써, 다양한 산업·탐사 환경으로 손쉽게 확장 가능

  1-6 기술 스택
  - 로봇 운영체제: ROS2 Humble, Micro-ROS
  - SBC: Ubuntu 22.04, Python 3.10, ROS2 Humble
  - MCU: Stm32CubeIDE, FreeRTOS, C Language, Micro-ROS Client
  - 센서 제어 라이브러리: OpenCV, Numpy, rclpy, geometry_msgs, sensor_msgs
  - 통신 및 네트워크: UART + DMA, UDP(Socket), Micro-ROS
  - 모터 제어: Dynamixel SDK, PWM Servo Control, DXL Protocol 1.0 / 2.0
  - 컨트롤러 App: Kotlin, WebView, UDP Socket API
  - 컨트롤러 서버: FastAPI, Socket/UDP 통신
 
##  팀원 소개
| <img src="https://github.com/user-attachments/assets/8e5e423b-1111-4a3c-b9aa-07b2a408f16d" width="120"/> | <img src="https://github.com/user-attachments/assets/f416b168-266b-4240-b8c1-e7dfd30d3414" width="120"/> | <img src="https://github.com/user-attachments/assets/60e12eac-3b9f-4781-8b41-2ab1d0c19d86" width="120"/> | <img src="https://github.com/user-attachments/assets/26db2e23-494c-41e0-a326-225555ba5bcf" width="120"/> | <img src="img/kpm.png" width="120"/> | 
|:--:|:--:|:--:|:--:|:--:|
| **권성진** <br> • 회로도 설계 및 구현 <br> • DXL Protocol 1.0 / 2.0 분리 제어 개발 <br> • Wheel-Mode 모터 제어 <br> • STM32 펌웨어 개발 | **김환희** <br> • ROS2 노드 및 소프트웨어 개발 <br> • Micro-ROS 통신 및 제어 로직 구현 <br> • STM32 펌웨어 개발 및 연동 <br> • 컨트롤러 앱 / 서버 개발 <br> • 장애물 회피 및 단차 인식 알고리즘 개발 <br> • LEG 모터 제어 | **김찬희** <br> • 로봇 몸체 설계 및 출력 <br> • 다리 구조 역기구학 계산 <br> • 시뮬레이션 및 보행 검증 | **박태영** <br> • 로봇 외형 및 프레임 설계 <br> • 하드웨어 출력 및 조립 <br> • 문서 작성 및 데이터 정리 | **김평무** <br> • 프로젝트 멘토링 <br> • 기술 자문 <br> • 현업 트렌드 기반 개선 방향 제시 <br> • 현실적인 해결방안 제시|

## 시스템 구상도
 서비스 구상도
 <img width="1130" height="535" alt="서비스 구상도" src="https://github.com/user-attachments/assets/513494b5-15bf-483b-b9e4-72b27f5efd22" />

  하드웨어 구상도
  <img width="1161" height="1103" alt="HW구상도" src="https://github.com/user-attachments/assets/7420b4c0-602b-406a-b21c-723a1c29078b" />

  소프트웨어 구상도
<img width="1581" height="711" alt="KakaoTalk_20251011_162127584_04 (1)" src="https://github.com/user-attachments/assets/22d91f0c-b9af-444b-ade7-4b703ce462be" />

  주행 흐름도
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
