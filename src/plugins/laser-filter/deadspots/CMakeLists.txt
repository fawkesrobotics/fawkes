set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${FAWKES_EXECUTABLE_DIR})
set(EXECUTABLE_fflaser_deadspots
    ON
    CACHE BOOL ON)

if(EXECUTABLE_fflaser_deadspots)
  add_executable(fflaser_deadspots deadspots.cpp)
  target_link_libraries(
    fflaser_deadspots
    stdc++
    m
    fawkescore
    fawkesutils
    fawkesnetcomm
    fawkesblackboard
    fawkesinterface
    fawkesconfig
    Laser360Interface
    Laser720Interface)
else()
  message(STATUS "Skip building disabled fflaser_deadspots executable")
endif()
