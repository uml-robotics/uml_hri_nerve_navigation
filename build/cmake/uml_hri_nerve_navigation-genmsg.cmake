# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uml_hri_nerve_navigation: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iuml_hri_nerve_navigation:/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uml_hri_nerve_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" NAME_WE)
add_custom_target(_uml_hri_nerve_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uml_hri_nerve_navigation" "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" "geometry_msgs/Pose2D:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" NAME_WE)
add_custom_target(_uml_hri_nerve_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uml_hri_nerve_navigation" "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" "geometry_msgs/Pose2D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uml_hri_nerve_navigation
)
_generate_msg_cpp(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uml_hri_nerve_navigation
)

### Generating Services

### Generating Module File
_generate_module_cpp(uml_hri_nerve_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uml_hri_nerve_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uml_hri_nerve_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uml_hri_nerve_navigation_generate_messages uml_hri_nerve_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_cpp _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_cpp _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uml_hri_nerve_navigation_gencpp)
add_dependencies(uml_hri_nerve_navigation_gencpp uml_hri_nerve_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uml_hri_nerve_navigation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uml_hri_nerve_navigation
)
_generate_msg_eus(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uml_hri_nerve_navigation
)

### Generating Services

### Generating Module File
_generate_module_eus(uml_hri_nerve_navigation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uml_hri_nerve_navigation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uml_hri_nerve_navigation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uml_hri_nerve_navigation_generate_messages uml_hri_nerve_navigation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_eus _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_eus _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uml_hri_nerve_navigation_geneus)
add_dependencies(uml_hri_nerve_navigation_geneus uml_hri_nerve_navigation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uml_hri_nerve_navigation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uml_hri_nerve_navigation
)
_generate_msg_lisp(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uml_hri_nerve_navigation
)

### Generating Services

### Generating Module File
_generate_module_lisp(uml_hri_nerve_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uml_hri_nerve_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uml_hri_nerve_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uml_hri_nerve_navigation_generate_messages uml_hri_nerve_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_lisp _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_lisp _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uml_hri_nerve_navigation_genlisp)
add_dependencies(uml_hri_nerve_navigation_genlisp uml_hri_nerve_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uml_hri_nerve_navigation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uml_hri_nerve_navigation
)
_generate_msg_nodejs(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uml_hri_nerve_navigation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(uml_hri_nerve_navigation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uml_hri_nerve_navigation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(uml_hri_nerve_navigation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(uml_hri_nerve_navigation_generate_messages uml_hri_nerve_navigation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_nodejs _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_nodejs _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uml_hri_nerve_navigation_gennodejs)
add_dependencies(uml_hri_nerve_navigation_gennodejs uml_hri_nerve_navigation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uml_hri_nerve_navigation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uml_hri_nerve_navigation
)
_generate_msg_py(uml_hri_nerve_navigation
  "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uml_hri_nerve_navigation
)

### Generating Services

### Generating Module File
_generate_module_py(uml_hri_nerve_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uml_hri_nerve_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uml_hri_nerve_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uml_hri_nerve_navigation_generate_messages uml_hri_nerve_navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/SimLog.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_py _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/csrobot/catkin_ws/src/uml_hri_nerve_navigation/msg/Goal.msg" NAME_WE)
add_dependencies(uml_hri_nerve_navigation_generate_messages_py _uml_hri_nerve_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uml_hri_nerve_navigation_genpy)
add_dependencies(uml_hri_nerve_navigation_genpy uml_hri_nerve_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uml_hri_nerve_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uml_hri_nerve_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uml_hri_nerve_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uml_hri_nerve_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uml_hri_nerve_navigation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uml_hri_nerve_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uml_hri_nerve_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uml_hri_nerve_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/uml_hri_nerve_navigation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uml_hri_nerve_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uml_hri_nerve_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uml_hri_nerve_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(uml_hri_nerve_navigation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
