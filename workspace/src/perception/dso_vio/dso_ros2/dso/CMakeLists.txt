cmake_minimum_required(VERSION 3.5)
project(dso)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer -lboost_thread"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DSO_PATH "/Users/sriramashokkumar/Documents/sbel/dso")
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)

add_executable(dso_ros src/dso_ros.cpp)
target_include_directories(dso_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus)
ament_target_dependencies(
  dso_ros
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "Boost"
  "Eigen3"
  "Pangolin"
)
target_link_libraries(dso_ros ${DSO_LIBRARY} ${PCL_LIBRARIES})

add_executable(camera src/camera.cpp)
target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  camera
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)

install(TARGETS dso_ros camera
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY calibration
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
