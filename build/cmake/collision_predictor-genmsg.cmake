# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "collision_predictor: 4 messages, 0 services")

set(MSG_I_FLAGS "-Icollision_predictor:/home/dikshant/catkin_ws/src/collision_predictor/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(collision_predictor_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" NAME_WE)
add_custom_target(_collision_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collision_predictor" "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" "geometry_msgs/Point:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/PoseStamped:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/TwistStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" NAME_WE)
add_custom_target(_collision_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collision_predictor" "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" "std_msgs/Bool"
)

get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" NAME_WE)
add_custom_target(_collision_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collision_predictor" "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" "geometry_msgs/PoseWithCovariance:std_msgs/Bool:nav_msgs/Odometry:geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:std_msgs/String:geometry_msgs/Twist:std_msgs/Header:std_msgs/Float32:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" NAME_WE)
add_custom_target(_collision_predictor_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collision_predictor" "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" "geometry_msgs/PoseWithCovariance:std_msgs/Bool:nav_msgs/Odometry:geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:std_msgs/String:geometry_msgs/Quaternion:geometry_msgs/Twist:std_msgs/Header:std_msgs/Float32:collision_predictor/VehicleState"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor
)
_generate_msg_cpp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor
)
_generate_msg_cpp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor
)
_generate_msg_cpp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor
)

### Generating Services

### Generating Module File
_generate_module_cpp(collision_predictor
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(collision_predictor_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(collision_predictor_generate_messages collision_predictor_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_cpp _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_cpp _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_cpp _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_cpp _collision_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_predictor_gencpp)
add_dependencies(collision_predictor_gencpp collision_predictor_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_predictor_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor
)
_generate_msg_eus(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor
)
_generate_msg_eus(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor
)
_generate_msg_eus(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor
)

### Generating Services

### Generating Module File
_generate_module_eus(collision_predictor
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(collision_predictor_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(collision_predictor_generate_messages collision_predictor_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_eus _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_eus _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_eus _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_eus _collision_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_predictor_geneus)
add_dependencies(collision_predictor_geneus collision_predictor_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_predictor_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor
)
_generate_msg_lisp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor
)
_generate_msg_lisp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor
)
_generate_msg_lisp(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor
)

### Generating Services

### Generating Module File
_generate_module_lisp(collision_predictor
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(collision_predictor_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(collision_predictor_generate_messages collision_predictor_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_lisp _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_lisp _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_lisp _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_lisp _collision_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_predictor_genlisp)
add_dependencies(collision_predictor_genlisp collision_predictor_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_predictor_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor
)
_generate_msg_nodejs(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor
)
_generate_msg_nodejs(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor
)
_generate_msg_nodejs(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor
)

### Generating Services

### Generating Module File
_generate_module_nodejs(collision_predictor
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(collision_predictor_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(collision_predictor_generate_messages collision_predictor_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_nodejs _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_nodejs _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_nodejs _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_nodejs _collision_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_predictor_gennodejs)
add_dependencies(collision_predictor_gennodejs collision_predictor_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_predictor_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor
)
_generate_msg_py(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor
)
_generate_msg_py(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor
)
_generate_msg_py(collision_predictor
  "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32.msg;/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor
)

### Generating Services

### Generating Module File
_generate_module_py(collision_predictor
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(collision_predictor_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(collision_predictor_generate_messages collision_predictor_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/waypoint.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_py _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/collision.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_py _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/VehicleState.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_py _collision_predictor_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dikshant/catkin_ws/src/collision_predictor/msg/Environment.msg" NAME_WE)
add_dependencies(collision_predictor_generate_messages_py _collision_predictor_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_predictor_genpy)
add_dependencies(collision_predictor_genpy collision_predictor_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_predictor_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_predictor
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(collision_predictor_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(collision_predictor_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(collision_predictor_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_predictor
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(collision_predictor_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(collision_predictor_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(collision_predictor_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_predictor
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(collision_predictor_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(collision_predictor_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(collision_predictor_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_predictor
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(collision_predictor_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(collision_predictor_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(collision_predictor_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_predictor
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(collision_predictor_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(collision_predictor_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(collision_predictor_generate_messages_py nav_msgs_generate_messages_py)
endif()
