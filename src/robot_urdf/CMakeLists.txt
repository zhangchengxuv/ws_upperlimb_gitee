cmake_minimum_required(VERSION 3.8)
project(robot_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

install(
  DIRECTORY launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

add_executable(joint_deg_gui src/joint_deg_gui_cpp.cpp)
add_executable(joint_deg_rightgui src/joint_deg_gui_right_cpp.cpp)
add_executable(joint_deg_robgui src/joint_deg_gui_rob_cpp.cpp)



ament_target_dependencies(joint_deg_gui
  rclcpp
  sensor_msgs
)
ament_target_dependencies(joint_deg_rightgui
  rclcpp
  sensor_msgs
)
ament_target_dependencies(joint_deg_robgui
  rclcpp
  sensor_msgs
)

target_link_libraries(joint_deg_gui
  Qt5::Widgets
)
target_link_libraries(joint_deg_rightgui
  Qt5::Widgets
)
target_link_libraries(joint_deg_robgui
  Qt5::Widgets
)

install(
  TARGETS joint_deg_gui joint_deg_rightgui joint_deg_robgui
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()