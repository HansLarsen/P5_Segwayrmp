cmake_minimum_required(VERSION 3.5)
project(segwayrmp2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(segwayinterfacemsgs REQUIRED)
find_package(segwayrmp REQUIRED)


set(dependencies
  builtin_interfaces
  geometry_msgs
  nav_msgs
  message_filters
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  segwayinterfacemsgs
  segwayrmp
)


link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
set(SEGWAYRMP_LINK_LIBS ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

list(APPEND SEGWAYRMP_LINK_LIBS
      dl
      rt
      pthread
    )

include_directories(${segwayrmp_INCLUDE_DIRS})
include_directories(${segwayinterfacemsgs_INCLUDE_DIRS})

add_executable(segway_node src/segway_rmp_node.cpp)
ament_target_dependencies(segway_node ${dependencies})

target_link_libraries(segway_node ${rclcpp_LIBRARIES} ${segwayrmp_LIBRARIES} ${SEGWAYRMP_LINK_LIBS})

install(TARGETS
	segway_node
	DESTINATION lib/${PROJECT_NAME}
	)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
