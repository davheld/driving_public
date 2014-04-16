cmake_minimum_required(VERSION 2.8.3)
project(blf)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  stdr_lib stdr_msgs sensor_msgs velodyne_msgs stdr_velodyne
  velodyne_pointcloud ladybug_playback
)
find_package(Boost REQUIRED COMPONENTS iostreams)

catkin_package(
  CATKIN_DEPENDS stdr_lib stdr_msgs sensor_msgs velodyne_msgs velodyne_pointcloud stdr_velodyne ladybug_playback
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/blf.cpp
  src/convert.cpp
  src/vlf.cpp
  src/llf.cpp

  # including header files too so that they appear nicely in QtCreator projects
  include/${PROJECT_NAME}/convert.h
  include/${PROJECT_NAME}/blf.h
  include/${PROJECT_NAME}/vlf.h
  include/${PROJECT_NAME}/llf.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_read_llf src/test_read_llf.cpp)
target_link_libraries(test_read_llf ${PROJECT_NAME})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_velodyne_conversion src/test_conversion.cpp)
  target_link_libraries(test_velodyne_conversion ${PROJECT_NAME})
endif()
