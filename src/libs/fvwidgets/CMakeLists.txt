add_library(
  fvwidgets SHARED image_widget.cpp fuse_image_list_widget.cpp
  # qa/qa_image_display.cpp
)
optional_depend_on_pkgconfig_libs(fvwidgets sdl sdl_dep_found)
if(sdl_dep_found)
  target_sources(fvwidgets PRIVATE sdl_keeper.cpp image_display.cpp)
else()
  build_skipped_message("fvwidgets SDL Widgets" "sdl12-compat[-devel]")
endif()

depend_on_fvconf(fvwidgets)
depend_on_gui(fvwidgets)
target_link_libraries(fvwidgets fawkescore fawkesutils fvutils)
