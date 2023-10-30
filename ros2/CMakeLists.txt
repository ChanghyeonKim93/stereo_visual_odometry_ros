message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_ros2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories ("/usr/include/opencv4/")
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

set(SRC ${SRC}
  wrapper/stereo_visual_odometry_ros2.cpp)

add_executable(stereo_visual_odometry_node wrapper/stereo_visual_odometry_node.cpp ${SRC})
target_link_libraries(stereo_visual_odometry_node ${CMAKE_PROJECT_NAME}_core ${Eigen3_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(stereo_visual_odometry_node rclcpp sensor_msgs geometry_msgs nav_msgs message_filters )
install(TARGETS stereo_visual_odometry_node
  DESTINATION lib/${PROJECT_NAME}) # install executable
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}) # install launch files
