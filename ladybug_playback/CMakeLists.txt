cmake_minimum_required(VERSION 2.8.3)
project(ladybug_playback)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp nodelet camera_info_manager image_transport dynamic_reconfigure
  stdr_lib stdr_msgs timer fastjpeg
)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
find_package(DC1394 REQUIRED)

generate_dynamic_reconfigure_options(cfg/LadybugRecombine.cfg)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet camera_info_manager image_transport 
                 dynamic_reconfigure stdr_lib stdr_msgs timer fastjpeg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${DC1394_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/frame_separator.h
  src/separator/pgr_compressor_header_info.h
  src/separator/pgr_compressor_header_info.cpp
  src/separator/frame_separator.cpp
  src/separator/frame_separator_base.cpp
  src/separator/frame_separator_base.h
  src/separator/frame_separator_nodelet.cpp

  include/${PROJECT_NAME}/recombine.h
  src/recombine/recombine.cpp
  src/recombine/recombine_node_base.cpp
  src/recombine/recombine_node_base.h
  src/recombine/recombine_nodelet.cpp

  include/${PROJECT_NAME}/subscriber.h
  src/subscriber.cpp
)
add_definitions("-fopenmp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${DC1394_LIBRARIES} ${OpenCV_LIBRARIES})
if(NOT APPLE)
target_link_libraries(${PROJECT_NAME} gomp)
endif()

# Make sure stdr_msgs messages are generated first
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

# Make sure that the dynamic reconfigure files are generated first
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(frame_separator_node src/separator/frame_separator_node.cpp)
target_link_libraries(frame_separator_node ${PROJECT_NAME})

add_executable(separated_frames_publisher src/separator/separated_frames_publisher.cpp)
target_link_libraries(separated_frames_publisher ${PROJECT_NAME})

add_executable(recombine_node src/recombine/recombine_node.cpp)
target_link_libraries(recombine_node ${PROJECT_NAME})
add_dependencies(recombine_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
