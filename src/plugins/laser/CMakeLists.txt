set(PLUGIN_laser
    ON
    CACHE BOOL ON)
include(laser)
if(PLUGIN_laser)
  if(URG_ROOT
     OR ORG_GBX_FOUND
     OR LIBUSB_FOUND)

    set(laser_SOURCES)
    list(APPEND laser_SOURCES laser_plugin.cpp acquisition_thread.cpp
         sensor_thread.cpp)
    if(URG_ROOT)
      list(APPEND laser_SOURCES urg_aqt.cpp)
    else()
      message(
        WARNING "No support for Hokuyo URG (urg[-devel] dependency missing)")
    endif()
    if(URG_GBX_FOUND)
      list(APPEND laser_SOURCES urg_gbx_aqt.cpp)
    else()
      message(
        WARNING
          "No support for Hokuyo via Gearbox library (flexiport[-devel] or hokuyoaist[-devel] dependency missing)"
      )
    endif()

    if(LIBUSB_FOUND OR SICK_TIM55X_BOOST_DEPS_FOUND)
      list(APPEND laser_SOURCES sick_tim55x_common_aqt.cpp)
      if(LIBUSB_FOUND)
        list(APPEND laser_SOURCES sick_tim55x_usb_aqt.cpp)
      else()
        message(
          WARNING
            "No support for Sick TiM55x-USB$ (libusb1[-devel] dependency missing)"
        )
      endif()

      if(SICK_TIM55X_BOOST_DEPS_FOUND)
        list(APPEND laser_SOURCES sick_tim55x_ethernet_aqt.cpp)
      else()
        message(
          WARNING
            "No suport for TiM55x-Ethernet (Boost system or thread dependency missing)"
        )
      endif()
    endif()
    add_library(laser MODULE ${laser_SOURCES})
    depend_on_laser(laser)
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
    message(WARNING "Omitting Laser Plugin (No hardware access library found)")
  endif()
else()
  message(STATUS "Skip building disabled laser plugin")
endif()
