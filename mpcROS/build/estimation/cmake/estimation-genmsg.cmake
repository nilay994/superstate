# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "estimation: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iestimation:/home/mavlab/develop/superstate/mpcROS/src/estimation/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(estimation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" NAME_WE)
add_custom_target(_estimation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "estimation" "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" "std_msgs/String"
)

get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" NAME_WE)
add_custom_target(_estimation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "estimation" "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" "estimation/IRMarker:std_msgs/Header:std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/estimation
)
_generate_msg_cpp(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/estimation
)

### Generating Services

### Generating Module File
_generate_module_cpp(estimation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/estimation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(estimation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(estimation_generate_messages estimation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" NAME_WE)
add_dependencies(estimation_generate_messages_cpp _estimation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(estimation_generate_messages_cpp _estimation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(estimation_gencpp)
add_dependencies(estimation_gencpp estimation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS estimation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/estimation
)
_generate_msg_eus(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/estimation
)

### Generating Services

### Generating Module File
_generate_module_eus(estimation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/estimation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(estimation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(estimation_generate_messages estimation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" NAME_WE)
add_dependencies(estimation_generate_messages_eus _estimation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(estimation_generate_messages_eus _estimation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(estimation_geneus)
add_dependencies(estimation_geneus estimation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS estimation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/estimation
)
_generate_msg_lisp(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/estimation
)

### Generating Services

### Generating Module File
_generate_module_lisp(estimation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/estimation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(estimation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(estimation_generate_messages estimation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" NAME_WE)
add_dependencies(estimation_generate_messages_lisp _estimation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(estimation_generate_messages_lisp _estimation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(estimation_genlisp)
add_dependencies(estimation_genlisp estimation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS estimation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/estimation
)
_generate_msg_nodejs(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/estimation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(estimation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/estimation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(estimation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(estimation_generate_messages estimation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" NAME_WE)
add_dependencies(estimation_generate_messages_nodejs _estimation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(estimation_generate_messages_nodejs _estimation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(estimation_gennodejs)
add_dependencies(estimation_gennodejs estimation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS estimation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/estimation
)
_generate_msg_py(estimation
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/estimation
)

### Generating Services

### Generating Module File
_generate_module_py(estimation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/estimation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(estimation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(estimation_generate_messages estimation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarker.msg" NAME_WE)
add_dependencies(estimation_generate_messages_py _estimation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mavlab/develop/superstate/mpcROS/src/estimation/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(estimation_generate_messages_py _estimation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(estimation_genpy)
add_dependencies(estimation_genpy estimation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS estimation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/estimation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/estimation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(estimation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/estimation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/estimation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(estimation_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/estimation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/estimation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(estimation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/estimation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/estimation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(estimation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/estimation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/estimation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/estimation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(estimation_generate_messages_py std_msgs_generate_messages_py)
endif()
