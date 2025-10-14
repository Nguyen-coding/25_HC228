cmake_minimum_required(VERSION 3.5)
project(hot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  
find_package(rosidl_default_generators REQUIRED)

# 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StepEvent.msg"
)

install(PROGRAMS

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
