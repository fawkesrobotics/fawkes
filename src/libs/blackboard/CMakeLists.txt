include_directories(libs)
add_library(
  fawkesblackboard SHARED
  interface_observer.cpp
  net/interface_observer.cpp
  net/interface_listener.cpp
  net/ilist_content.cpp
  net/interface_proxy.cpp
  net/handler.cpp
  shmem/lister.cpp
  shmem/header.cpp
  interface_list_maintainer.cpp
  internal/memory_manager.cpp
  internal/interface_manager.cpp
  internal/notifier.cpp
  internal/instance_factory.cpp
  internal/message_manager.cpp
  # qa/qa_bb_buffers.cpp
  # qa/qa_bb_notify.cpp
  # qa/qa_bb_listall.cpp
  # qa/qa_bb_messaging.cpp
  # qa/qa_bb_remote.cpp
  # qa/qa_bb_openall.cpp
  # qa/qa_bb_objpos.cpp
  # qa/qa_bb_memmgr.cpp
  # qa/qa_bb_interface.cpp
  interface_listener.cpp
  utils/on_message_waker.cpp
  utils/on_update_waker.cpp
  remote.cpp
  local.cpp
  blackboard.cpp
  ownership.cpp)

# TODO: link to std++ is missing from binary?!
target_link_libraries(fawkesblackboard fawkescore fawkesutils fawkesnetcomm
                      fawkeslogging fawkesinterface)

set(LUA_SOURCES blackboard.tolua)
set(LUA_LINK_LIBS fawkescore fawkesutils fawkesblackboard)
generate_lua_lib(fawkesblackboard "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
