cmake_minimum_required(VERSION 2.8.3)
project(stdr_velodyne)
set(CMAKE_BUILD_TYPE RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS 
  std_msgs roscpp nodelet dynamic_reconfigure 
  velodyne_pointcloud stdr_msgs stdr_lib bag_of_tricks
)
find_package(PCL REQUIRED COMPONENTS common)

generate_dynamic_reconfigure_options(cfg/Aggregator.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs roscpp nodelet velodyne_pointcloud stdr_msgs stdr_lib bag_of_tricks
  DEPENDS PCL
  CFG_EXTRAS stdr_velodyne-extras.cmake
)

include(${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/stdr_velodyne-extras.cmake)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/stdr_velodyne/conversion.h
  src/conversion/conversion.cpp
  src/conversion/conversion_node_base.cpp
  src/conversion/conversion_node_base.h
  src/conversion/conversion_nodelet.cpp
  src/conversion/conversion_node.cpp

  include/stdr_velodyne/point_type.h
  include/stdr_velodyne/config.h
  include/stdr_velodyne/pointcloud.h
  src/pointcloud/config.cpp
  src/pointcloud/defaultcal.h
  src/pointcloud/pointcloud.cpp
  src/pointcloud/pointcloud_node_base.h
  src/pointcloud/pointcloud_node_base.cpp
  src/pointcloud/pointcloud_nodelet.cpp

  include/stdr_velodyne/message_filter.h
  src/message_filter.cpp
  include/stdr_velodyne/transform.h
  src/transform.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Make sure stdr_msgs messages are generated first
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

add_dependencies(${PROJECT_NAME} velodyne_generate_messages_cpp)


add_executable(conversion src/conversion/conversion_node.cpp)
target_link_libraries(conversion ${PROJECT_NAME})

add_executable(pointcloud src/pointcloud/pointcloud_node.cpp)
target_link_libraries(pointcloud ${PROJECT_NAME})

add_executable(stdr_velodyne_aggregator_node src/aggregator_node.cpp)
target_link_libraries(stdr_velodyne_aggregator_node ${PROJECT_NAME})
set_target_properties(stdr_velodyne_aggregator_node
                      PROPERTIES OUTPUT_NAME aggregator_node
                      PREFIX "")

# make sure configure headers are built before using them
add_dependencies(aggregator_node ${PROJECT_NAME}_gencfg)
