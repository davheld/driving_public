cmake_minimum_required(VERSION 2.8.3)
project(track_file_tools)
set(CMAKE_BUILD_TYPE Debug)

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

add_executable(track_file_tools_stats src/stats.cpp)
target_link_libraries(track_file_tools_stats ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(track_file_tools_stats
                      PROPERTIES OUTPUT_NAME stats
                      PREFIX "")

include_directories(${VTK_INCLUDE_DIRS})
add_executable(track_file_tools_visualizer src/visualizer.cpp)
target_link_libraries(track_file_tools_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${VTK_LIBRARIES})
set_target_properties(track_file_tools_visualizer
                      PROPERTIES OUTPUT_NAME visualizer
                      PREFIX "")

add_executable(track_file_tools_aggregator src/aggregator.cpp)
target_link_libraries(track_file_tools_aggregator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(track_file_tools_aggregator
                      PROPERTIES OUTPUT_NAME aggregator
                      PREFIX "")

add_executable(track_file_tools_cumulator src/cumulator.cpp)
target_link_libraries(track_file_tools_cumulator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(track_file_tools_cumulator
                      PROPERTIES OUTPUT_NAME cumulator
                      PREFIX "")
