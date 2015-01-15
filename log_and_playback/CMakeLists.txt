cmake_minimum_required(VERSION 2.8.3)
project(log_and_playback)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp rosbag_storage urdf kdl_parser tf_conversions
  stdr_msgs stdr_lib blf dgc_transform localize stdr_velodyne ladybug_playback
)

find_package(Boost REQUIRED COMPONENTS iostreams program_options)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rosbag_storage urdf kdl_parser tf_conversions
    stdr_msgs stdr_lib blf dgc_transform localize stdr_velodyne ladybug_playback
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/bag_reader.cpp
  src/bag_tf_listener.cpp
  src/dgclogio.cpp
  src/dgclog_reader.cpp
  src/data_reader.cpp
  src/kittireader.cpp
  src/robot_model.cpp
  src/spinello.cpp
  src/spin_reader.cpp

  include/log_and_playback/abstract_data_reader.h
  include/log_and_playback/bag_reader.h
  include/log_and_playback/bag_tf_listener.h
  include/log_and_playback/data_reader.h
  include/log_and_playback/dgclog_reader.h
  include/log_and_playback/dgclogio.h
  include/log_and_playback/kittireader.h
  include/log_and_playback/robot_model.h
  include/log_and_playback/spinello.h
  include/log_and_playback/spin_reader.h
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Make sure stdr_msgs messages are generated first
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(example_spin_reader src/example_spin_reader.cpp)
target_link_libraries(example_spin_reader ${PROJECT_NAME})

add_executable(log_playback src/log_playback.cpp)
target_link_libraries(log_playback ${PROJECT_NAME})

add_executable(bag2dgc src/bag_to_dgc_converter.cpp)
target_link_libraries(bag2dgc ${PROJECT_NAME})

add_executable(split_bags src/split_bags.cpp)
target_link_libraries(split_bags ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
