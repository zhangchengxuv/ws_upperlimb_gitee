cmake_minimum_required(VERSION 3.8)
project(upperlimb_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# Dependencies
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ========================================
# ROS2 Interfaces
# ========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/RobotState.msg"
  "msg/SystemState.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# ========================================
# Library paths
# ========================================
set(GOOGOLTECH_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(GOOGOLTECH_LIBS
  ${GOOGOLTECH_LIB_DIR}/libgxn_x64.so
  ${GOOGOLTECH_LIB_DIR}/libgt_rn_x64.so
  ${GOOGOLTECH_LIB_DIR}/libGLink_x64.so
  ${GOOGOLTECH_LIB_DIR}/libMachineBuilding_x64.so
  ${GOOGOLTECH_LIB_DIR}/libLAFunc_x64.so
  ${GOOGOLTECH_LIB_DIR}/libecat_master.so
  ${GOOGOLTECH_LIB_DIR}/libecat_config.so
  ${GOOGOLTECH_LIB_DIR}/libecfg_parser.so
)

# ========================================
# Executables
# ========================================
add_executable(motor_driver_node
  src/motor_driver_node.cpp
)

add_executable(control_algorithm_node
  src/control_algorithm_node.cpp
)

# ========================================
# Include directories
# ========================================
target_include_directories(motor_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(control_algorithm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========================================
# Compile features
# ========================================
target_compile_features(motor_driver_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(control_algorithm_node PUBLIC c_std_99 cxx_std_17)

# ========================================
# Ament dependencies
# ========================================
ament_target_dependencies(motor_driver_node
  rclcpp
  std_msgs
  ament_index_cpp
)

ament_target_dependencies(control_algorithm_node
  rclcpp
  std_msgs
)

# ========================================
# ROSIDL type support
# ========================================
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ========================================
# Link libraries
# ========================================
target_link_libraries(motor_driver_node
  "${cpp_typesupport_target}"
  ${GOOGOLTECH_LIBS}
)

target_link_libraries(control_algorithm_node
  "${cpp_typesupport_target}"
  ${GOOGOLTECH_LIBS}
)

# ========================================
# Install
# ========================================
install(TARGETS
  motor_driver_node
  control_algorithm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

install(
  DIRECTORY lib/
  DESTINATION share/${PROJECT_NAME}/lib
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES config/Gecat.eni
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ========================================
# Testing
# ========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
