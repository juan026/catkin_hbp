cmake_minimum_required(VERSION 2.8.3)
project(gazebo_msgs2)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
# NRP custom messages
  JointStates.msg
  OpenSimPhysics.msg
  WheelSpeeds.msg
  SoundWithSource.msg
  )

add_service_files(DIRECTORY srv FILES
# NRP custom services
  AdvanceSimulation.srv
  DeleteLight.srv
  ExportWorldSDF.srv
  GetLightProperties.srv
  GetLightsName.srv
  GetVisualProperties.srv
  SetLightProperties.srv
  SetVisualProperties.srv
  SetSensorNoiseProperties.srv
  GetSensorNoiseProperties.srv
  SetHuskyCmdVel.srv
  GetHuskyOdometry.srv
  GetHuskyJointStates.srv
  SetHuskyWheelSpeeds.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  nav_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  )
