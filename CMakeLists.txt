cmake_minimum_required(VERSION 2.8.3)
project(beggar_bot)
find_package(catkin REQUIRED COMPONENTS 
  roscpp std_msgs genmsg message_generation)

add_message_files(
  FILES
  SensorData.msg
  DetectionBox.msg
  DetectionList.msg
)
  
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} /usr/local/include)
link_directories(/usr/local/lib)

add_executable(switch_node src/switch.cpp)
target_link_libraries(switch_node ${catkin_LIBRARIES} wiringPi)
add_dependencies(switch_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sensor_node src/sensor.cpp submodules/mcp3008/mcp3008Spi.cpp)
target_link_libraries(sensor_node ${catkin_LIBRARIES})
add_dependencies(sensor_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(camera_node src/camera.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_generate_messages_cpp)
