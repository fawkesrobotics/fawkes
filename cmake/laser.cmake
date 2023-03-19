pkg_check_modules(FLEXIPORT flexiport)

pkg_check_modules(HOKUYOAIST hokuyoaist)
if(FLEXIPORT_FOUND AND HOKUYOAIST_FOUND)
  set(URG_GBX_FOUND 1)
  set(URG_GBX_C_DEFINITION HAVE_URG_GBX)
endif()

pkg_check_modules(LIBUDEV libudev)
if(LIBUDEV_FOUND)
  set(LIBUDEV_C_DEFINITION HAVE_LIBUDEV)
endif()

pkg_check_modules(LIBUSB libusb-1.0)
if(LIBUSB_FOUND)
  set(LIBUSB_C_DEFINITION HAVE_LIBUSB)
endif()

# TODO libpcan not available on fedora?

find_package(Boost COMPONENTS thread system)
if(Boost_FOUND)
  set(SICK_TIM55X_BOOST_DEPS_FOUND 1)
  add_library(sick_tim55x INTERFACE IMPORTED)
  target_include_directories(sick_tim55x INTERFACE ${Boost_INCLUDE_DIRS})
  target_link_libraries(sick_tim55x INTERFACE -lpthread ${Boost_LIBRARIES})
  target_compile_definitions(sick_tim55x INTERFACE HAVE_SICK55X_BOOST)
  # target_compile_definitions do not work with imported targets
  set(SICK_TIM55X_C_DEFINITION HAVE_SICK55X_BOOST)
endif()

# urg-devel
find_path(URG_ROOT urg/UrgCtrl.h PATH_SUFFIXES include)
if(URG_ROOT)
  add_library(liburg INTERFACE IMPORTED)
  target_include_directories(liburg INTERFACE ${URG_ROOT})
  target_link_directories(liburg INTERFACE ${URG_ROOT}/lib)
  target_link_libraries(
    liburg
    INTERFACE urg
              urg_connection
              urg_monitor
              urg_coordinate
              urg_geometry
              urg_system
              urg_common
              urg_connection_sdl)
  set(URG_C_DEFINITION HAVE_URG)
endif()

function(depend_on_laser target)
  # if clang: CFLAGS_URG_GBX += -Wno-overloaded-virtual
  target_link_libraries(${target} ${FLEXIPORT_LDFLAGS} ${HOKUYOAIST_LDFLAGS}
                        ${LIBUDEV_LDFLAGS} ${LIBUSB_LDFLAGS})
  target_compile_options(
    ${target} PUBLIC ${FLEXIPORT_CFLAGS} ${HOKUYOAIST_CFLAGS} ${LIBUDEV_CFLAGS}
                     ${LIBUSB_CFLAGS})
  target_compile_definitions(
    ${target}
    PUBLIC ${URG_GBX_C_DEFINITION} ${LIBUSB_C_DEFINITION}
           ${LIBUDEV_C_DEFINITION} ${SICK_TIM55X_C_DEFINITION}
           ${URG_C_DEFINITION})
  if(URG_ROOT)
    target_link_libraries(${target} liburg)
  endif()
  if(SICK_TIM55X_BOOST_DEPS_FOUND)
    target_link_libraries(${target} sick_tim55x)
  endif()
endfunction()
