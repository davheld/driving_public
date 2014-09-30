cmake_minimum_required(VERSION 2.8.3)
project(track_file_io)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs sensor_msgs rosbag_storage rosconsole)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES
  Frame.msg
  Track.msg
  Tracks.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs rosbag_storage rosconsole
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/track_file_io/manipulations.h
  include/track_file_io/track_file_io.h
  src/manipulations.cpp
  src/track_file_io.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  find_package(PCL REQUIRED COMPONENTS common)
  find_package(pcl_conversions REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS} ${pcl_conversions_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS} ${pcl_conversions_LIBRARY_DIRS})

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(test_track_file_io src/test_track_file_io.cpp)
  target_link_libraries(test_track_file_io ${PROJECT_NAME} ${PCL_LIBRARIES} ${pcl_conversions_LIBRARIES})
endif()
