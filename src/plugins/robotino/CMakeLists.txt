set(PLUGIN_robotino
    ON
    CACHE BOOL ON)

if(PLUGIN_robotino)
  find_package(Boost COMPONENTS thread filesystem)
  add_subdirectory(interfaces)

  if(LIBUDEV_FOUND
     AND Boost_FOUND
     AND CPP_17_FOUND)
    add_library(
      robotino MODULE
      robotino_plugin.cpp com_thread.cpp sensor_thread.cpp act_thread.cpp
      direct_com_thread.cpp direct_com_message.cpp)
    # openrobotino_com_thread.cpp # if openrobotino is available
    target_compile_definitions(robotino PUBLIC HAVE_ROBOTINO_DIRECT)
    # c++ 11

    target_compile_features(robotino PUBLIC cxx_std_11)
    target_link_libraries(robotino ${LIBUDEV_LDFLAGS})
    target_compile_options(robotino PUBLIC ${LIBUDEV_CFLAGS})
    target_compile_definitions(robotino PUBLIC ${LIBUDEV_C_DEFINITION})

    if(TF_FOUND)
      depend_on_tf(robotino)
      set_source_files_properties(
        sensor_thread.cpp PROPERTIES COMPILE_OPTIONS
                                     "-Wno-reorder -Wno-unused-function")
    endif()

    target_link_libraries(
      robotino
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesbaseapp
      fawkesblackboard
      fawkesinterface
      MotorInterface
      GripperInterface
      BatteryInterface
      RobotinoSensorInterface
      IMUInterface)

  else()
    message(WARNING "Omitting Plugin robotino")
    if(NOT FLITE_ROOT)
      message(WARNING "robotino[-devel] dependency missing")
    endif()
    if(NOT ALSA_FOUND)
      message(WARNING "alsa-lib[-devel] dependency missing")
    endif()
  endif()
else()
  plugin_disabled_message(robotino)
endif()
