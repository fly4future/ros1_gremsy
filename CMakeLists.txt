cmake_minimum_required(VERSION 2.8.3)
project(ros1_gremsy)
set(CMAKE_CXX_STANDARD 17)

set(CATKIN_DEPENDENCIES
    cmake_modules
    roscpp
    nodelet
    message_generation
    tf2
    tf2_geometry_msgs
    dynamic_reconfigure
    sensor_msgs
    mrs_msgs
    geometry_msgs
    message_generation
    message_runtime
)

set(LIBRARIES
  GimbalControllerNode
  )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ROSGremsy.cfg
)

add_service_files(DIRECTORY srv FILES
  SetGimbalAttitude.srv
)
# messages
add_message_files(DIRECTORY msg FILES
  GimbalDiagnostics.msg
  )
generate_messages(
    DEPENDENCIES
    geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

include_directories(
 src/gSDK/src/mavlink/include/mavlink/v2.0
 src/gSDK/src/
)

add_library(GremsyDriverNode
    src/ros1_gremsy_nodelet.cpp
    src/gSDK/src/gimbal_interface.cpp
    src/gSDK/src/serial_port.cpp
    src/gSDK/src/gimbal_protocol.cpp
    src/gSDK/src/gimbal_protocol_v1.cpp
    src/gSDK/src/gimbal_protocol_v2.cpp
)

set(SOURCES 
  src/ros1_gremsy.cpp 
  src/gSDK/src/serial_port.cpp 
  src/gSDK/src/gimbal_interface.cpp
  src/gSDK/src/gimbal_protocol.cpp
  src/gSDK/src/gimbal_protocol_v1.cpp
  src/gSDK/src/gimbal_protocol_v2.cpp
)

target_include_directories(GremsyDriverNode PUBLIC
  src/gSDK/src/mavlink/include/mavlink/v2.0
  src/gSDK/src
)

add_executable(GimbalNode ${SOURCES})

add_dependencies(GimbalNode ${PROJECT_NAME}_gencfg)

target_link_libraries(GimbalNode ${catkin_LIBRARIES})

target_link_libraries(GremsyDriverNode
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(TARGETS GremsyDriverNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
