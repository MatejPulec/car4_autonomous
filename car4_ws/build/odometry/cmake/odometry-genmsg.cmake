# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "odometry: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iodometry:/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(odometry_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" NAME_WE)
add_custom_target(_odometry_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odometry" "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(odometry
  "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odometry
)

### Generating Services

### Generating Module File
_generate_module_cpp(odometry
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odometry
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(odometry_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(odometry_generate_messages odometry_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" NAME_WE)
add_dependencies(odometry_generate_messages_cpp _odometry_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odometry_gencpp)
add_dependencies(odometry_gencpp odometry_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odometry_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(odometry
  "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odometry
)

### Generating Services

### Generating Module File
_generate_module_eus(odometry
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odometry
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(odometry_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(odometry_generate_messages odometry_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" NAME_WE)
add_dependencies(odometry_generate_messages_eus _odometry_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odometry_geneus)
add_dependencies(odometry_geneus odometry_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odometry_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(odometry
  "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odometry
)

### Generating Services

### Generating Module File
_generate_module_lisp(odometry
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odometry
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(odometry_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(odometry_generate_messages odometry_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" NAME_WE)
add_dependencies(odometry_generate_messages_lisp _odometry_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odometry_genlisp)
add_dependencies(odometry_genlisp odometry_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odometry_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(odometry
  "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odometry
)

### Generating Services

### Generating Module File
_generate_module_nodejs(odometry
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odometry
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(odometry_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(odometry_generate_messages odometry_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" NAME_WE)
add_dependencies(odometry_generate_messages_nodejs _odometry_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odometry_gennodejs)
add_dependencies(odometry_gennodejs odometry_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odometry_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(odometry
  "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odometry
)

### Generating Services

### Generating Module File
_generate_module_py(odometry
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odometry
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(odometry_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(odometry_generate_messages odometry_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mrmat420/car4_autonomous/car4_ws/src/odometry/msg/CarState.msg" NAME_WE)
add_dependencies(odometry_generate_messages_py _odometry_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odometry_genpy)
add_dependencies(odometry_genpy odometry_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odometry_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odometry)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odometry
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(odometry_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odometry)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odometry
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(odometry_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odometry)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odometry
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(odometry_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odometry)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odometry
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(odometry_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odometry)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odometry\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odometry
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(odometry_generate_messages_py std_msgs_generate_messages_py)
endif()
