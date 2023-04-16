include_directories(libs)
add_library(
  fawkesconfig SHARED
  memory.cpp
  change_handler.cpp
  # qa/qa_net_list_content.cpp
  # qa/qa_config_change_handler.cpp
  # qa/qa_yaml.cpp
  # qa/qa_config.cpp
  fawkesconfig_tolua.cpp
  net_list_content.cpp
  sqlite.cpp
  netconf.cpp
  net_handler.cpp
  config.cpp
  yaml.cpp)
if(EXISTS "${CMAKE_SYSROOT}/usr/include/sys/inotify.h")
  target_compile_options(fawkesconfig PUBLIC -DHAVE_INOTIFY)
endif()
depend_on_yamlcpp(fawkesconfig)

target_link_libraries(fawkesconfig stdc++ fawkescore fawkesutils fawkesnetcomm
                      fawkeslogging)

depend_on_pkgconfig_libs(fawkesconfig sqlite3)
target_compile_options(fawkesconfig PUBLIC -DHAVE_SQLITE)

set(LUA_SOURCES config.tolua)
set(LUA_LINK_LIBS fawkescore fawkesconfig)
generate_lua_lib(fawkesconfig "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
