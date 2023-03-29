set(PLUGIN_clips-pddl-parser
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-pddl-parser)
  pkg_check_modules(pddl_parser pddl_parser)
  if(pddl_parser_FOUND)
    if(CLIPSMM_FOUND)

      add_library(
        clips-pddl-parser MODULE
        clips_pddl_parser_plugin.cpp clips_pddl_parser_thread.cpp
        clips_pddl_parser_feature.cpp precondition_visitor.cpp
        effect_visitor.cpp)
      target_link_libraries(clips-pddl-parser ${pddl_parser_LDFLAGS})
      depend_on_clipsmm(clips-pddl-parser)
      target_compile_options(clips-pddl-parser PUBLIC ${pddl_parser_CFLAGS})
      target_link_libraries(clips-pddl-parser fawkescore fawkesclipsaspect)
    else()

    endif()
  endif()
else()
  plugin_disabled_message("clips-pddl-parser")
endif()
