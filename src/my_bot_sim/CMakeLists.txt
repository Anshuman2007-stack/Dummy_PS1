cmake_minimum_required(VERSION 3.8)
project(my_bot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models urdf
  DESTINATION share/${PROJECT_NAME})

# `ros2 run my_bot_sim motor_driver_sim.py`
install(PROGRAMS scripts/motor_driver_sim.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
