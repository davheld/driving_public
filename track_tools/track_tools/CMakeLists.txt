cmake_minimum_required(VERSION 2.8.3)
project(track_tools)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs dgc_transform)
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs dgc_transform
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  include/track_tools/track_manager.h
  src/track_manager.cpp
)
add_definitions("-Wno-deprecated-declarations")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(track_stats src/prog/track_stats.cpp)
target_link_libraries(track_stats ${PROJECT_NAME})

include_directories(${VTK_INCLUDE_DIRS})
add_executable(track_visualizer src/prog/track_visualizer.cpp)
target_link_libraries(track_visualizer ${PROJECT_NAME} ${PCL_VISUALIZATION_LIBRARIES} ${VTK_LIBRARIES})

add_executable(track_aggregator src/prog/track_aggregator.cpp)
target_link_libraries(track_aggregator ${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(track_cumulator src/prog/track_cumulator.cpp)
target_link_libraries(track_cumulator ${PROJECT_NAME} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
