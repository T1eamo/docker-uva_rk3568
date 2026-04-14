cmake_minimum_required(VERSION 3.8)
project(uva_pkg)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

set(UVA_VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(UVA_UTILS_DIR ${UVA_VENDOR_DIR}/utils)
set(UVA_STB_DIR ${UVA_VENDOR_DIR}/stb)
set(RKNN_RT_LIB ${UVA_VENDOR_DIR}/librknnrt.so)
set(JPEG_TURBO_INCLUDE_DIR ${UVA_VENDOR_DIR}/jpeg_turbo/include)
set(JPEG_TURBO_LIBRARY ${UVA_VENDOR_DIR}/jpeg_turbo/lib/libturbojpeg.a)
set(UVA_UTILS_LIBRARIES
  ${UVA_UTILS_DIR}/libfileutils.a
  ${UVA_UTILS_DIR}/libimagedrawing.a
  ${UVA_UTILS_DIR}/libimageutils.a
)

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

foreach(required_path
    ${RKNN_RT_LIB}
    ${JPEG_TURBO_LIBRARY}
    ${UVA_UTILS_LIBRARIES})
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Missing required vendored dependency: ${required_path}")
  endif()
endforeach()

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
  message(WARNING
    "uva_pkg uses bundled RKNN/RGA ARM64 binaries. Native builds on ${CMAKE_SYSTEM_PROCESSOR} "
    "are expected to fail at link or runtime; use the provided ARM64 Dockerfile or build on the target device.")
endif()

add_executable(uva_node
  src/main.cpp
  src/test.cpp
  src/postprocess.cpp
  src/yolov5.cpp
)
ament_target_dependencies(uva_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(uva_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UVA_UTILS_DIR}
  ${UVA_STB_DIR}
  ${JPEG_TURBO_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${RGA_INCLUDE_DIR}
)

target_compile_definitions(uva_node PRIVATE
  UVA_PKG_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(uva_node
  ${RKNN_RT_LIB}
  ${UVA_UTILS_LIBRARIES}
  ${JPEG_TURBO_LIBRARY}
  ${RGA_LIBRARY}
  ${OpenCV_LIBS}
  Threads::Threads
)

set_target_properties(uva_node PROPERTIES
  BUILD_RPATH "${UVA_VENDOR_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS
  uva_node
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ${RKNN_RT_LIB}
  DESTINATION lib
)
install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
