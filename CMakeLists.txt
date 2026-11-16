cmake_minimum_required(VERSION 3.15)
project(cern_shared_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(VISP REQUIRED COMPONENTS core)

set(CMAKE_CXX_STANDARD 17)

# main node: shared control (teleoperation + inspection + safety layer)
# the robot_*.cpp files are copies of my models from ecn_manip (Exercises 1-3)
ament_auto_add_executable(shared_control
  src/shared_control.cpp
  src/robot_kr16.cpp
  src/robot_turret.cpp
  src/robot_ur10.cpp)
target_include_directories(shared_control PUBLIC ${VISP_INCLUDE_DIRS})
target_link_libraries(shared_control ${VISP_LIBRARIES})

ament_auto_package()
