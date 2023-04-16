set(EXECUTABLE_skillet
    ON
    CACHE BOOL ON)

if(EXECUTABLE_skillet)
  add_executable(skillet skillet.cpp)
  depend_on_pkgconfig_libs(skillet "ncurses;readline")
  target_link_libraries(
    skillet
    stdc++
    fawkescore
    fawkesutils
    fawkesnetcomm
    fawkesblackboard
    fawkesinterface
    readline
    history
    SkillerInterface)
else()
  executable_disabled_message(skillet)
endif()
