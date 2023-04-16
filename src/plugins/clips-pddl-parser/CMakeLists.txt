set(PLUGIN_clips-pddl-parser
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-pddl-parser)
  add_library(
    clips-pddl-parser MODULE
    clips_pddl_parser_plugin.cpp clips_pddl_parser_thread.cpp
    clips_pddl_parser_feature.cpp precondition_visitor.cpp effect_visitor.cpp)
  depend_on_pkgconfig_libs(clips-pddl-parser pddl_parser)
  depend_on_clipsmm(clips-pddl-parser)
  target_link_libraries(clips-pddl-parser fawkescore fawkesclipsaspect)
else()
  plugin_disabled_message("clips-pddl-parser")
endif()
