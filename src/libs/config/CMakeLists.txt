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

pkg_check_modules(sqlite3 REQUIRED sqlite3)
target_compile_options(fawkesconfig PUBLIC -DHAVE_SQLITE ${sqlite3_CFLAGS})
target_link_libraries(fawkesconfig ${sqlite3_LDFLAGS})

include(lua)
generate_tolua(fawkesconfig config.tolua)
add_library(fawkesconfig_tolua SHARED fawkesconfig_tolua.cpp)
depend_on_lua(fawkesconfig_tolua)
target_link_libraries(fawkesconfig_tolua fawkescore fawkesconfig)
