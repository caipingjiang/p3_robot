cmake_minimum_required(VERSION 3.5)
project(p3dx_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
