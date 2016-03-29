cmake_minimum_required(VERSION 2.8.3)

project(realsense_camera)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
# Flags executables
set(CMAKE_EXE_LINKER_FLAGS "-pie -z noexecstack -z relro -z now")
# Flags shared libraries
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  nodelet
  cv_bridge
  image_transport
  camera_info_manager
  tf
  message_generation
  std_msgs
  sensor_msgs
  rostest
  pcl_ros
  librealsense
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/camera_params.cfg
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure roscpp nodelet cv_bridge image_transport camera_info_manager tf
    message_generation std_msgs sensor_msgs pcl_ros librealsense
    LIBRARIES realsense_camera_nodelet
    )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(realsense_camera_nodelet src/realsense_camera_driver.cpp src/realsense_camera_nodelet.cpp)
target_link_libraries(realsense_camera_nodelet 
  ${catkin_LIBRARIES}
  #/usr/local/lib/librealsense.so
)
add_dependencies(realsense_camera_nodelet realsense_camera_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(realsense_camera_node src/realsense_camera_driver.cpp src/realsense_camera_node.cpp )
target_link_libraries(realsense_camera_node
  ${catkin_LIBRARIES}
)
add_dependencies(realsense_camera_node realsense_camera_generate_messages_cpp ${PROJECT_NAME}_gencfg)
 
install(TARGETS realsense_camera_nodelet realsense_camera_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    ) 

install(FILES realsense_camera_nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

