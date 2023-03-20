set(PLUGIN_laser-filter
    ON
    CACHE BOOL ON)
add_subdirectory(deadspots)

if(PLUGIN_laser-filter)
  add_subdirectory(interfaces)

  set(laser-filter_SOURCES)
  list(
    APPEND
    laser-filter_SOURCES
    filter_thread.cpp
    filters/deadspots.cpp
    filters/cascade.cpp
    filters/circle_sector.cpp
    filters/min_merge.cpp
    filters/reverse_angle.cpp
    filters/min_circle.cpp
    filters/filter.cpp
    filters/map_filter.cpp
    filters/720to360.cpp
    filters/1080to360.cpp
    filters/box_filter.cpp
    filters/copy.cpp
    filters/max_circle.cpp
    laser_filter_plugin.cpp)
  if(TF_FOUND)
    list(APPEND laser-filter_SOURCES filters/projection.cpp)
  else()
    message(
      WARNING
        "Omitting projection filter of laser-filter plugin bullet[-devel] dependency missing"
    )
  endif()
  add_library(laser-filter MODULE ${laser-filter_SOURCES})
  target_link_libraries(
    laser-filter
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkes_amcl_utils
    fawkes_amcl_map
    Laser360Interface
    Laser720Interface
    Laser1080Interface
    LaserBoxFilterInterface)
  if(TF_FOUND)
    depend_on_tf(laser-filter)
  endif()

else()
  message(STATUS "Skip building disabled laser-filter plugin")
endif()
