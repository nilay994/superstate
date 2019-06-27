cmake_minimum_required(VERSION 2.8.3)
project(controllermavlab)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs mav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp std_msgs mav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/attitude_control.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(DIRECTORY
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
