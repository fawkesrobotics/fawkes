if(HAVE_GUI)
  set(fvwidgets_SOURCES)
  list(APPEND fvwidgets_SOURCES image_widget.cpp fuse_image_list_widget.cpp
       # qa/qa_image_display.cpp
  )
  if(SDL_FOUND)
    list(APPEND fvwidgets_SOURCES sdl_keeper.cpp image_display.cpp)
  else()
    message(
      WARNING
        "Omitting FireVision SDL Widgets (sdl12-compat[-devel] not installed)")
  endif()

  add_library(fvwidgets SHARED ${fvwidgets_SOURCES})
  depend_on_fvconf(fvwidgets)
  depend_on_gui(fvwidgets)
  target_link_libraries(fvwidgets fawkescore fawkesutils fvutils)
else()
  message(
    WARNING "Omitting FireVision Widgets (GUI dependencies not installed)")
endif()
