cmake_minimum_required(VERSION 3.8)
project(ros2_audio_pub)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioWithDOA.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
