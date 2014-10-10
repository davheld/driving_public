cmake_minimum_required(VERSION 2.8.3)
project(localize)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf diagnostic_updater stdr_msgs global_coords
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED COMPONENTS highgui)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES localize
  CATKIN_DEPENDS roscpp tf diagnostic_updater stdr_msgs global_coords
  DEPENDS Boost
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/localize/applanix_transformer.h
  include/localize/fake_localizer.h
  include/localize/localizer_base.h

  src/applanix_transformer.cpp
  src/fake_localizer.cpp
  src/localizer_base.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Make sure stdr_msgs messages are generated first
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(smooth_odom_tf_publisher src/smooth_odom_tf_publisher.cpp)
target_link_libraries(smooth_odom_tf_publisher ${PROJECT_NAME})

add_executable(fake_localizer src/fake_localizer_node.cpp)
target_link_libraries(fake_localizer ${PROJECT_NAME})

add_executable(localize_pose_tf_broadcaster src/localize_pose_tf_broadcaster.cpp)
target_link_libraries(localize_pose_tf_broadcaster ${PROJECT_NAME})

add_executable(gps_fix_pub src/gps_fix_pub.cpp)
target_link_libraries(gps_fix_pub ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
