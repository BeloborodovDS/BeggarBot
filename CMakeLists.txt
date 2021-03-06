SET(OPENVINO_PATH "/home/pi/Work/libs/inference_engine_vpu_arm")

SET(OPENVINO_LIBS "${OPENVINO_PATH}/deployment_tools/inference_engine/lib/raspbian_9/armv7l")
SET(OPENVINO_INCLUDE "${OPENVINO_PATH}/deployment_tools/inference_engine/include")

cmake_minimum_required(VERSION 2.8.3)
project(beggar_bot)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  std_srvs
  genmsg 
  message_generation
  roslib)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  SensorData.msg
  DetectionBox.msg
  DetectionList.msg)
  
add_service_files(
  FILES
  ServoAction.srv
  ServoSpeed.srv
  ServoHeadPlatform.srv)
  
generate_messages(DEPENDENCIES 
    std_msgs
    std_srvs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    include
    /usr/local/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OPENVINO_INCLUDE})
link_directories(
    /usr/local/lib
    ${OPENVINO_LIBS})

# SWITCH NODE
add_executable(switch_node 
    src/switch.cpp)
target_compile_options(switch_node 
    PRIVATE 
    -march=armv7-a 
    -std=c++11 
    -O3 
    -Wno-psabi)
target_link_libraries(switch_node 
    ${catkin_LIBRARIES} 
    wiringPi)
add_dependencies(switch_node 
    ${PROJECT_NAME}_generate_messages_cpp)

# SENSORS NODE
add_executable(sensor_node 
    src/sensor.cpp 
    submodules/mcp3008/mcp3008Spi.cpp)
target_compile_options(sensor_node 
    PRIVATE 
    -march=armv7-a 
    -std=c++11 
    -O3 
    -Wno-psabi)
target_link_libraries(sensor_node 
    ${catkin_LIBRARIES})
add_dependencies(sensor_node 
    ${PROJECT_NAME}_generate_messages_cpp)

# CAMERA + NCS NODE
add_executable(camera_node 
    src/camera.cpp 
    src/ncs_wrapper/vino_wrapper.cpp
    submodules/sort-cpp/sort-c++/SORTtracker.cpp
    submodules/sort-cpp/sort-c++/Hungarian.cpp
    submodules/sort-cpp/sort-c++/KalmanTracker.cpp)
target_compile_options(camera_node 
    PRIVATE 
    -march=armv7-a 
    -std=c++11 
    -O3 
    -Wno-psabi)
target_link_libraries(camera_node 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    raspicam
    dl
    inference_engine)
add_dependencies(camera_node 
    ${PROJECT_NAME}_generate_messages_cpp)
    
# SERVO NODE
add_executable(servo_node 
    src/servo.cpp)
target_compile_options(servo_node 
    PRIVATE 
    -march=armv7-a 
    -std=c++11 
    -O3 
    -Wno-psabi)
target_link_libraries(servo_node 
    ${catkin_LIBRARIES}
    wiringPi
    wiringPiPca9685)
add_dependencies(servo_node 
    ${PROJECT_NAME}_generate_messages_cpp)
    
# MAIN NODE
add_executable(beggar_bot 
    src/robot.cpp)
target_compile_options(beggar_bot 
    PRIVATE 
    -march=armv7-a 
    -std=c++11 
    -O3 
    -Wno-psabi)
target_link_libraries(beggar_bot 
    ${catkin_LIBRARIES})
add_dependencies(beggar_bot 
    ${PROJECT_NAME}_generate_messages_cpp)
