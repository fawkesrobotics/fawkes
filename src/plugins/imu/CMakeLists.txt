set(PLUGIN_imu
    ON
    CACHE BOOL ON)

if(PLUGIN_imu)

  set(SOURCES imu_plugin.cpp acquisition_thread.cpp sensor_thread.cpp)

  find_package(Boost COMPONENTS thread asio system)
  if(Boost_FOUND)
    list(APPEND SOURCES imu_cruizcore_xg1010.cpp)
    set(CRUIZCORE 1)
  endif()
  add_library(imu MODULE ${SOURCES})
  if(CRUIZCORE)
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
