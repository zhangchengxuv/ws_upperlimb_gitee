cmake_minimum_required(VERSION 3.8)
project(upperlimb_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/RobotState.msg"
  "msg/SystemState.msg"
)

# 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

# 可执行文件
add_executable(motor_driver_node src/motor_driver_node.cpp)

target_include_directories(motor_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(motor_driver_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(motor_driver_node
  rclcpp
  std_msgs
)

# 关键：把生成的消息类型支持链接给节点
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 链接库
target_link_libraries(motor_driver_node
  "${cpp_typesupport_target}"
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libgxn_x64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libgt_rn_x64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libGLink_x64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libMachineBuilding_x64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libLAFunc_x64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libecat_master.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libecat_config.so
  ${CMAKE_CURRENT_SOURCE_DIR}/include/upperlimb_robot/libecfg_parser.so
)

install(TARGETS
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()