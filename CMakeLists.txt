cmake_minimum_required(VERSION 3.22)
project(kuka_moveit2_spike)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf meshes config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
