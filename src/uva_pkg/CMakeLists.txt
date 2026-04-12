cmake_minimum_required(VERSION 3.8)
project(uva_pkg)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_path(RGA_INCLUDE_DIR
  NAMES im2d.hpp
  PATHS /usr/local/include /usr/include
  REQUIRED
)

find_library(RGA_LIBRARY
  NAMES rga librga
  PATHS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu
  REQUIRED
)

add_executable(uva_node src/main.cpp src/test.cpp)
ament_target_dependencies(uva_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(uva_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${RGA_INCLUDE_DIR}
)

target_link_libraries(uva_node
  ${RGA_LIBRARY}
  ${OpenCV_LIBS}
)

install(TARGETS
  uva_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
