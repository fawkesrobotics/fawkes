set(EXECUTABLE_skillet
    ON
    CACHE BOOL ON)

pkg_check_modules(ncurses ncurses)
pkg_check_modules(readline readline)
if(EXECUTABLE_skillet)
  if(ncurses_FOUND)
    if(readline_FOUND)

      add_executable(skillet skillet.cpp)
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
        SkillerInterface
        ${ncurses_LDFLAGS})
    else()
      build_skipped_message(skillet "readline[-devel]")
    endif()
  else()
    build_skipped_message(skillet "ncurses[-devel]")
  endif()
else()
  executable_disabled_message(skillet)
endif()
