cmake_minimum_required(VERSION 3.5)
project(hot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # ✅ 올바른 이름
find_package(rosidl_default_generators REQUIRED)

# 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StepEvent.msg"
)

# ✅ Python 노드 실행을 위한 설정
# 실행 가능한 스크립트 (setup.py의 console_scripts에 따라 자동 링크됨)
install(PROGRAMS
  # 아래 경로는 setup.py의 console_scripts와 일치해야 함
  # build/install 구조 기준으로 hot/ 하위에 실제 있는 실행 스크립트를 지정
  hot/step_detection/step.py
  hot/shock.py
  hot/server.py
  hot/lidar.py
  hot/data.py
  hot/lidar_data.py
  hot/camera.py
  hot/lidar2.py
  hot/stm.py
  hot/auto.py
  hot/avoid.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
