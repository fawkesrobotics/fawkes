set(PLUGIN_laser
    ON
    CACHE BOOL ON)
if(PLUGIN_laser)
  add_library(laser MODULE laser_plugin.cpp acquisition_thread.cpp
                           sensor_thread.cpp)
  depend_on_laser(laser)
  if(URG_ROOT)
    target_sources(laser PRIVATE urg_aqt.cpp)
  else()
    build_skipped_message("laser plugin URG support" "urg[-devel] dependency")
  endif()
  if(URG_GBX_FOUND)
    target_sources(laser PRIVATE urg_gbx_aqt.cpp)
  else()
    build_skipped_message("laser plugin Hokuyo via Gearbox support"
                          "flexiport[-devel] or hokuyoaist[-devel] dependency")
  endif()

  if(LIBUSB_FOUND OR SICK_TIM55X_BOOST_DEPS_FOUND)
    target_sources(laser PRIVATE sick_tim55x_common_aqt.cpp)
    if(LIBUSB_FOUND)
      target_sources(laser PRIVATE sick_tim55x_usb_aqt.cpp)
    else()
      build_skipped_message("laser plugin Sick TiM55x-USB support"
                            "libusb1[-devel] dependency")
    endif()

    if(SICK_TIM55X_BOOST_DEPS_FOUND)
      target_sources(laser PRIVATE sick_tim55x_ethernet_aqt.cpp)
    else()
      build_skipped_message("laser plugin TiM55x-Ethernet support "
                            "Boost system or thread dependency")
    endif()
  endif()
  target_link_libraries(
    laser
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    Laser360Interface
    Laser720Interface
    Laser1080Interface)
else()
  plugin_disabled_message(laser)
endif()
