execute_process(COMMAND "/home/dikshant/catkin_ws/src/collision_predictor/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dikshant/catkin_ws/src/collision_predictor/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
