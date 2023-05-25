cmake_minimum_required(VERSION 3.8)
project(cone_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# TODO: add the other dependencies
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  message_filters
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  art_perception_msgs
  OpenCV
)

foreach (dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${DEPENDENCIES})


set(LIBRARIES
  ${OpenCV_LIBRARIES}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${LIBRARIES} ${OpenCV_LIBRARIES})

rclcpp_components_register_nodes(${PROJECT_NAME} "ConeDetector")

ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)


ament_package()
