set(PLUGIN_laser-filter
    ON
    CACHE BOOL ON)
add_subdirectory(deadspots)
add_subdirectory(interfaces)

if(PLUGIN_laser-filter)

  add_library(
    laser-filter MODULE
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
  optional_depend_on_pkgconfig_libs(laser-filter "${tf_deps}" tf_deps_found)
  if(tf_deps_found)
    target_sources(laser-filter PRIVATE filters/projection.cpp)
    depend_on_tf(laser-filter)
  else()
    build_skipped_message("projection filter of laser-filter plugin"
                          "${tf_deps}")
  endif()
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

else()
  plugin_disabled_message(laser-filter)
endif()
