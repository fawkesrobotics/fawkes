add_library(fawkesinterface SHARED field_iterator.cpp interface.cpp message.cpp
                                   interface_info.cpp message_queue.cpp)
target_link_libraries(fawkesinterface fawkescore fawkesutils)
target_include_directories(
  fawkesinterface
  PUBLIC
  interface)
find_package(PkgConfig REQUIRED)
set(LUA_SOURCES interface.tolua message.tolua message_queue.tolua)
set(LUA_LINK_LIBS)
generate_lua_lib(fawkesinterface "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
depend_on_lua(fawkesinterface_tolua)
