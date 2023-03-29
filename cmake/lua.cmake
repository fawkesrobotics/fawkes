pkg_check_modules(LUA REQUIRED lua-5.1)

find_program(TOLUAPP tolua++ REQUIRED)

function(depend_on_lua target)
  set(LUADIR ${PROJECT_SOURCE_DIR}/src/lua)
  set(LUALIBDIR ${PROJECT_SOURCE_DIR}/lib/lua)
  # TODO This was EXEC_BASEDIR
  set(EXEC_LUADIR ${PROJECT_SOURCE_DIR}/src/lua)
  # TODO This was EXEC_LIBDIR
  set(EXEC_LUALIBDIR ${PROJECT_SOURCE_DIR}/lib/lua)
  target_compile_options(
    ${target} PUBLIC ${LUA_CFLAGS} -DHAVE_LUA -DLUADIR=\"${EXEC_LUADIR}\"
                     -DLUALIBDIR=\"${EXEC_LUALIBDIR}\" -Wno-unused-function)
  target_link_libraries(${target} ${LUA_LDFLAGS} tolua++)
  # TODO Secondexpansion stuff from lua.mk
endfunction()

# TODO tolua 5.1 vs 5.4? TODO link against tolua++?
function(generate_tolua target)
  string(REPLACE ";" " " FILES "${ARGN}")
  add_custom_command(
    OUTPUT ${INTERFACE_PREFIX}${target}_tolua.pkg
           ${INTERFACE_PREFIX}${target}_tolua.cpp
    COMMAND
      /bin/sh -c
      "${PROJECT_SOURCE_DIR}/etc/scripts/./tolua_generate.sh ${CMAKE_CURRENT_BINARY_DIR} ${INTERFACE_PREFIX}${target} ${PROJECT_SOURCE_DIR}/src/lua/fawkes/toluaext.lua ${PROJECT_SOURCE_DIR}/doc/headers/lichead_c.GPL_WRE ${CMAKE_CURRENT_SOURCE_DIR} ${FILES}"
    VERBATIM
    DEPENDS ${ARGN} ${PROJECT_SOURCE_DIR}/etc/scripts/tolua_generate.sh)
endfunction()

function(generate_lua_lib target sources link_libs)
  generate_tolua(${target} "${sources}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FAWKES_LUA_LIB_DIR})
  add_library(${target}_tolua SHARED ${target}_tolua.cpp)
  set_target_properties(${target}_tolua PROPERTIES PREFIX "")
  depend_on_lua(${target}_tolua)
  target_link_libraries(${target}_tolua "${link_libs}")
  set_target_properties(${target}_tolua PROPERTIES OUTPUT_NAME ${target})
endfunction()

function(generate_interface_from_xml interface)
  get_filename_component(if_name ${interface} NAME_WLE)
  set(INTERFACE_PREFIX interfaces_)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.tolua
           ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.h
           ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp
    COMMAND ffifacegen -d ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/${interface}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${interface})
  generate_tolua(${if_name} ${if_name}.tolua)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FAWKES_LUA_INTERFACE_DIR})
  add_library(${if_name}_tolua SHARED ${INTERFACE_PREFIX}${if_name}_tolua.cpp)
  depend_on_lua(${if_name}_tolua)
  set_target_properties(${if_name}_tolua PROPERTIES PREFIX "")
  set_target_properties(${if_name}_tolua PROPERTIES OUTPUT_NAME ${if_name})
  set_target_properties(${if_name}_tolua PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                    ${FAWKES_LUA_INTERFACE_DIR})

  target_link_libraries(${if_name}_tolua fawkescore fawkesutils fawkesinterface
                        ${if_name})
  add_library(${if_name} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp)
  set_target_properties(${if_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                              ${FAWKES_INTERFACE_DIR})
  target_include_directories(${if_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)
  target_link_libraries(${if_name} fawkescore fawkesutils fawkesinterface)

  # target_link_libraries(fawkesutils_tolua fawkescore fawkesutils)
endfunction()
