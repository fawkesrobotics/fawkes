set(PLUGIN_joystick
    ON
    CACHE BOOL ON)

set(PLUGIN_joystick-teleop
    ON
    CACHE BOOL ON)

set(PLUGIN_ffjoystick
    ON
    CACHE BOOL ON)

set(PLUGIN_ffkbjoystick
    ON
    CACHE BOOL ON)

if(PLUGIN_joystick)
  add_library(
    joystick MODULE
    joystick_plugin.cpp sensor_thread.cpp acquisition_thread.cpp bb_handler.cpp
    force_feedback.cpp act_thread.cpp)
  target_link_libraries(
    joystick
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    JoystickInterface)
else()
  plugin_disabled_message(joystick)
endif()

if(PLUGIN_joystick-teleop)
  add_library(joystick-teleop MODULE joystick_teleop_plugin.cpp
                                     joystick_teleop_thread.cpp)
  target_link_libraries(
    joystick-teleop
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface)
else()
  plugin_disabled_message(joystick-teleop)
endif()

if(PLUGIN_ffjoystick)
  add_executable(
    ffjoystick ffjoystick.cpp acquisition_thread.cpp remote_bb_poster.cpp
               bb_handler.cpp force_feedback.cpp act_thread.cpp)
  target_link_libraries(
    ffjoystick
    stdc++
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkeslogging
    JoystickInterface)
else()
  plugin_disabled_message(ffjoystick)
endif()

if(PLUGIN_ffkbjoystick)
  add_executable(ffkbjoystick ffkbjoystick.cpp remote_bb_poster.cpp
                              bb_handler.cpp)
  target_link_libraries(
    ffkbjoystick
    stdc++
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkeslogging
    JoystickInterface)
else()
  plugin_disabled_message(ffkbjoystick)
endif()
