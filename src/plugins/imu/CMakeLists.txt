set(PLUGIN_imu
    ON
    CACHE BOOL ON)

if(PLUGIN_imu)
  add_library(imu MODULE imu_plugin.cpp acquisition_thread.cpp
                         sensor_thread.cpp)

  optional_depend_on_boost_libs(imu "thread;system" boost_deps_found)
  if(boost_deps_found)

    target_sources(imu PRIVATE imu_cruizcore_xg1010.cpp)
    target_link_libraries(imu ${Boost_LIBRARIES})
    target_include_directories(imu PRIVATE ${Boost_INCLUDE_DIRS})
    depend_on_tf(imu)
    target_compile_definitions(imu PUBLIC HAVE_CRUIZCORE)
  endif()

  target_link_libraries(
    imu
    m
    pthread
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    IMUInterface)

else()
  plugin_disabled_message("imu")
endif()
