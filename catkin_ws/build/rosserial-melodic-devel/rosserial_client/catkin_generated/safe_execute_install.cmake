execute_process(COMMAND "/home/marvin/mARVin/catkin_ws/build/rosserial-melodic-devel/rosserial_client/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/marvin/mARVin/catkin_ws/build/rosserial-melodic-devel/rosserial_client/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
