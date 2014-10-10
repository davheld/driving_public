cmake_minimum_required(VERSION 2.8.3)
project(dgc_transform)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS tf eigen_extensions stdr_lib)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tf eigen_extensions stdr_lib
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_library(${PROJECT_NAME}
  include/dgc_transform/dgc_transform.h
  src/dgc_transform.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(tfm_broadcaster src/tfm_broadcaster.cpp)
target_link_libraries(tfm_broadcaster ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
