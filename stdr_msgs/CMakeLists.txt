cmake_minimum_required(VERSION 2.8.3)
project(stdr_msgs)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs nav_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  ApplanixDMI.msg
  ApplanixGPS.msg
  ApplanixPose.msg
  ApplanixRMS.msg
  LocalizePose.msg

  LadybugImages.msg

  Actuator.msg
  CanStatus.msg
  EStopRequest.msg
  EStopStatus.msg
  HCIString.msg
  PassatExtOutput.msg
  PassatOutput.msg
  PassatState.msg
  PassatStatus.msg
  PointWithCurve.msg
  PointWithCurveArray.msg
  Trajectory2D.msg
  TrajectoryPoint2D.msg
  TurnSignal.msg

  DynamicObstacle.msg
  StaticObstaclePoint.msg
  PerceptionObstacles.msg
  LabeledTrack.msg

  TrafficLightState.msg
  TrafficLightPose.msg
  TrafficLightStates.msg
  TrafficLightStateRequest.msg

  # old messages
  RawScan.msg
  RawScans.msg
  VelConfig.msg
  VelStat.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  IgnoreROWVehicle.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs)


# -----------------------------------------------------------------------------
# Add the following lines to any package that depends on stdr_msgs:
#

# Make sure stdr_msgs messages are generated first
#if(catkin_EXPORTED_TARGETS)
#  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#endif()

#------------------------------------------------------------------------------
