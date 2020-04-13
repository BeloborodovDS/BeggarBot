cmake_minimum_required(VERSION 2.8.3)
project(beggar_bot)
find_package(catkin REQUIRED roscpp std_msgs genmsg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} /usr/local/include)
link_directories(/usr/local/lib)

add_executable(switch_node src/switch.cpp)
target_link_libraries(switch_node ${catkin_LIBRARIES} wiringPi)
add_dependencies(switch_node beggar_bot_generate_messages_cpp)
