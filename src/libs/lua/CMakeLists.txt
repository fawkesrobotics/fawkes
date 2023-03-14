include(lua)
if(LUA_FOUND)
  add_library(
    fawkeslua SHARED
    # qa/qa_context.cpp
    context_watcher.cpp trigger.cpp context.cpp exceptions.cpp
    interface_importer.cpp)
  target_link_libraries(
    fawkeslua
    fawkescore
    fawkesutils
    fawkesconfig
    fawkesblackboard
    fawkesinterface
    fawkeslogging)
  depend_on_lua(fawkeslua)
else()
  message(
    WARNING
      "Omitting Fawkes Lua integration$ (lua-5.1 or tolua++[-devel] not installed)"
  )
endif()
