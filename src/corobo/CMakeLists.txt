cmake_minimum_required(VERSION 3.8)
project(corobo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(corobo_arm_drive SHARED src/corobo_arm_drive.cpp)
target_include_directories(corobo_arm_drive PUBLIC ${GAZEBO_INCLUDE_DIRS})
ament_target_dependencies(corobo_arm_drive gazebo_dev gazebo_ros rclcpp geometry_msgs)
target_link_libraries(corobo_arm_drive ${GAZEBO_LIBRARIES})

install(TARGETS corobo_arm_drive
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY urdf worlds config launch rviz
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/latlon_goal.py
  scripts/latlon_odom.py
  scripts/serial_base.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
