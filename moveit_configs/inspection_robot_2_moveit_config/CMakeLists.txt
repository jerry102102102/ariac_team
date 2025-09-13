cmake_minimum_required(VERSION 3.5)
project(inspection_robot_2_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()