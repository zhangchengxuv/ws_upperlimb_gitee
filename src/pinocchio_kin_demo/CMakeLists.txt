cmake_minimum_required(VERSION 3.8)
project(pinocchio_kin_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(Python_EXECUTABLE ${Python3_EXECUTABLE})

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# =========================
# fk_rviz_node
# =========================
add_executable(fk_rviz_node src/fk_rviz_node.cpp)

target_include_directories(fk_rviz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fk_rviz_node
  rclcpp
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(fk_rviz_node
  pinocchio::pinocchio
)

# =========================
# ik_rviz_node
# =========================
add_executable(ik_rviz_node src/ik_rviz_node.cpp)

target_include_directories(ik_rviz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_rviz_node
  rclcpp
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(ik_rviz_node
  pinocchio::pinocchio
)

# =========================
# ik_jacobian_transpose_node
# =========================
add_executable(ik_jacobian_transpose_node src/ik_jacobian_transpose_node.cpp)

target_include_directories(ik_jacobian_transpose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ik_jacobian_transpose_node
  rclcpp
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(ik_jacobian_transpose_node
  pinocchio::pinocchio
)

install(TARGETS
  fk_rviz_node
  ik_rviz_node
  ik_jacobian_transpose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()