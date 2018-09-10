cmake_minimum_required(VERSION 2.8.3)
project(amazing_car)

set(PCL_DIR /usr/local/pcl-1.8.1/share/pcl-1.8)

add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
# Find catkin
#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
    FILES
    my_location_msg.msg
    my_angle_msg.msg
    my_gps_state.msg
    my_lidar_distance.msg
    my_server_cmd.msg
    my_checkpoints.msg
    my_car_state.msg
)

generate_messages(
     DEPENDENCIES
     std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES AI_car
   CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

if(APPLE)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(FOUNDATION_LIBRARY Foundation)
endif()

if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        DEPENDS rt pthread
    )
else()
    # Otherwise normal call
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
    )
endif()

## Sources
set(serial_SRCS
    src/serial.cc
    src/pid.cpp
    include/serial/serial.h
    include/serial/v8stdint.h
    include/pidClass.h
    include/wheelSpeed.h
    include/magicPoint.h
    include/magicVector.h
)
if(APPLE)
	# If OSX
	list(APPEND serial_SRCS src/impl/unix.cc)
	list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(${PROJECT_NAME} ${serial_SRCS})
if(APPLE)
	target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
	target_link_libraries(${PROJECT_NAME} rt pthread)
else()
	target_link_libraries(${PROJECT_NAME} setupapi)
endif()

## Uncomment for example
add_executable(joy_subscriber examples/joy_subscriber.cpp)
add_executable(serial_subscriber examples/serial_subscriber.cpp)
add_executable(serial_example examples/serial_example.cc)

add_executable(my_tar_location_publisher src/my_tar_location_publisher.cpp)
add_executable(algorithm src/algorithm.cpp)
add_executable(gprs_location_publisher src/gprs_location_publisher.cpp)
add_executable(my_big_car_controller src/my_big_car_controller.cpp)
add_executable(ui_transdata src/ui_transdata.cpp)
add_executable(gyroscope src/gyroscope.cpp)
add_executable(beixing_lidar src/beixing_lidar.cpp)
add_executable(vlp16_lidar src/vlp16_lidar.cpp)
add_executable(my_server src/my_server.cpp)


add_dependencies(serial_example ${PROJECT_NAME})
target_link_libraries(serial_example ${PROJECT_NAME})
target_link_libraries(serial_example
   ${catkin_LIBRARIES}
)

add_dependencies(serial_subscriber ${PROJECT_NAME})
target_link_libraries(serial_subscriber ${PROJECT_NAME})
target_link_libraries(serial_subscriber
  ${catkin_LIBRARIES}
)

add_dependencies(joy_subscriber ${PROJECT_NAME})
target_link_libraries(joy_subscriber ${PROJECT_NAME})
target_link_libraries(joy_subscriber
  ${catkin_LIBRARIES}
)


add_dependencies(gyroscope ${PROJECT_NAME})
target_link_libraries(gyroscope ${PROJECT_NAME})
target_link_libraries(gyroscope
  ${catkin_LIBRARIES}
)

add_dependencies(algorithm ${PROJECT_NAME})
target_link_libraries(algorithm ${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(algorithm
  ${catkin_LIBRARIES}
)

add_dependencies(my_tar_location_publisher ${PROJECT_NAME})
target_link_libraries(my_tar_location_publisher ${PROJECT_NAME})
target_link_libraries(my_tar_location_publisher
  ${catkin_LIBRARIES}
)

add_dependencies(gprs_location_publisher ${PROJECT_NAME})
target_link_libraries(gprs_location_publisher ${PROJECT_NAME})
target_link_libraries(gprs_location_publisher
  ${catkin_LIBRARIES}
)

add_dependencies(my_big_car_controller ${PROJECT_NAME})
target_link_libraries(my_big_car_controller ${PROJECT_NAME})
target_link_libraries(my_big_car_controller
  ${catkin_LIBRARIES}
)

add_dependencies(ui_transdata ${PROJECT_NAME})
target_link_libraries(ui_transdata ${PROJECT_NAME})
target_link_libraries(ui_transdata
  ${catkin_LIBRARIES}
)

add_dependencies(beixing_lidar ${PROJECT_NAME})
target_link_libraries(beixing_lidar ${PROJECT_NAME})
target_link_libraries(beixing_lidar
  ${catkin_LIBRARIES}
  pcap
)

add_dependencies(vlp16_lidar ${PROJECT_NAME})
target_link_libraries(vlp16_lidar ${PROJECT_NAME})
target_link_libraries(vlp16_lidar
  ${catkin_LIBRARIES}
  pcap
)

add_dependencies(my_server ${PROJECT_NAME})
target_link_libraries(my_server ${PROJECT_NAME})
target_link_libraries(my_server
  ${catkin_LIBRARIES}
)

## Include headers
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)

## Tests
if(CATKIN_ENABLE_TESTING)
    add_subdirectory(tests)
endif()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
