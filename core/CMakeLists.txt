message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_core)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories ("/usr/include/opencv4/")
include_directories(${Eigen3_INCLUDE_DIRS})

set(SRC ${SRC}
  stereo_visual_odometry.cpp
  optimization/summary.cc
)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

add_library(${CMAKE_PROJECT_NAME}_core STATIC ${SRC})
target_link_libraries(${CMAKE_PROJECT_NAME}_core ${Eigen3_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS ${CMAKE_PROJECT_NAME}_core 
  DESTINATION lib/${PROJECT_NAME}
  CONFIGURATIONS Release)
  
message("   -- [INFO   ]: shared library file name: lib${CMAKE_PROJECT_NAME}_core.a")
