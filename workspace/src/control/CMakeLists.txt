cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(chrono_ros_msgs REQUIRED)
find_package(art_msgs REQUIRED)

add_executable(control src/control.cpp)
target_include_directories(control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/control>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(control rclcpp geometry_msgs nav_msgs chrono_ros_msgs art_msgs)

install(TARGETS control
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Configure clang-format
set(CLANG_FORMAT_EXECUTABLE_PATH /usr/bin/clang-format)

# Checking if clang-format exists
if(NOT EXISTS "${CLANG_FORMAT_EXECUTABLE_PATH}")
    message(FATAL_ERROR "clang-format executable not found at ${CLANG_FORMAT_EXECUTABLE_PATH}.
    Please install clang-format or specify the correct path.")
endif()

# Define a custom target to format code using clang-format
add_custom_target(format
  COMMAND ${CLANG_FORMAT_EXECUTABLE_PATH} -style=file -i ${CMAKE_CURRENT_SOURCE_DIR}/src/control.cpp
  COMMAND ${CLANG_FORMAT_EXECUTABLE_PATH} -style=file -i ${CMAKE_CURRENT_SOURCE_DIR}/include/control/control.h
  COMMENT "Running clang-format"
)

# Hook the format target to the build process
add_dependencies(control format)

ament_package()
