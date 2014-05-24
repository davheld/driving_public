cmake_minimum_required(VERSION 2.8.3)
project(track_file_tools)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions track_file_io)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  CATKIN_DEPENDS roscpp pcl_conversions track_file_io
  DEPENDS Boost PCL
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/common.h
  src/common.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(track_file_tools_track_stats src/track_stats.cpp)
target_link_libraries(track_file_tools_track_stats ${PROJECT_NAME} ${PCL_LIBRARIES})
set_target_properties(track_file_tools_track_stats
                      PROPERTIES OUTPUT_NAME track_stats
                      PREFIX "")

include_directories(${VTK_INCLUDE_DIRS})
add_executable(track_file_tools_track_visualizer src/track_visualizer.cpp)
target_link_libraries(track_file_tools_track_visualizer ${PROJECT_NAME} ${PCL_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${VTK_LIBRARIES})
set_target_properties(track_file_tools_track_visualizer
                      PROPERTIES OUTPUT_NAME track_visualizer
                      PREFIX "")

add_executable(track_file_tools_track_aggregator src/track_aggregator.cpp)
target_link_libraries(track_file_tools_track_aggregator ${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(track_file_tools_track_aggregator
                      PROPERTIES OUTPUT_NAME track_aggregator
                      PREFIX "")

add_executable(track_file_tools_track_cumulator src/track_cumulator.cpp)
target_link_libraries(track_file_tools_track_cumulator ${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(track_file_tools_track_cumulator
                      PROPERTIES OUTPUT_NAME track_cumulator
                      PREFIX "")
