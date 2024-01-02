cmake_minimum_required(VERSION 3.5)
project(gnss_2_utm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(eco_ros_lib REQUIRED)  
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)

set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  
find_package(visualization_msgs REQUIRED)  
find_package(tf2_ros REQUIRED)  
find_package(tf2_geometry_msgs REQUIRED) 
 

add_executable(gnss_2_utm src/gnss_2_utm.cpp)
ament_target_dependencies(gnss_2_utm rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs)
#eco_ros_lib

target_link_libraries(gnss_2_utm
  Geographic
)
install(TARGETS
  gnss_2_utm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
