add_library(fawkesinterface SHARED field_iterator.cpp interface.cpp message.cpp
                                   interface_info.cpp message_queue.cpp)
target_link_libraries(fawkesinterface fawkescore fawkesutils)
target_include_directories(
  fawkesinterface
  PUBLIC
  interface)
find_package(PkgConfig REQUIRED)
include(lua)
generate_tolua(fawkesinterface interface.tolua message.tolua
               message_queue.tolua)
add_library(fawkesinterface_tolua SHARED fawkesinterface_tolua.cpp)
depend_on_lua(fawkesinterface_tolua)
