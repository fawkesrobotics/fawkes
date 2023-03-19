pkg_check_modules(LUA lua-5.1)

find_program(TOLUAPP tolua++)

function(depend_on_lua target)
  set(LUADIR $(PROJECT_SOURCE_DIR)/src/lua)
  set(LUALIBDIR $(CMAKE_INSTALL_FULL_LIBDIR)/lua)
  # TODO This was EXEC_BASEDIR
  set(EXEC_LUADIR $(PROJECT_SOURCE_DIR)/src/lua)
  # TODO This was EXEC_LIBDIR
  set(EXEC_LUALIBDIR $(CMAKE_INSTALL_FULL_LIBDIR)/lua)
  target_compile_options(
    ${target} PUBLIC ${LUA_CFLAGS} -DHAVE_LUA -DLUADIR=\"$(EXEC_LUADIR)\"
                     -DLUALIBDIR=\"$(EXEC_LUALIBDIR)\" -Wno-unused-function)
  target_link_libraries(${target} ${LUA_LDFLAGS} tolua++)
  # TODO Secondexpansion stuff from lua.mk
endfunction()

# TODO tolua 5.1 vs 5.4? TODO link against tolua++?
function(generate_tolua target)
  string(REPLACE ";" " " FILES "${ARGN}")
  add_custom_command(
    OUTPUT ${target}_tolua.pkg ${target}_tolua.cpp
    COMMAND
      /bin/sh -c
      "${PROJECT_SOURCE_DIR}/etc/scripts/./tolua_generate.sh ${CMAKE_CURRENT_BINARY_DIR} ${target} ${PROJECT_SOURCE_DIR}/src/lua/fawkes/toluaext.lua ${PROJECT_SOURCE_DIR}/doc/headers/lichead_c.GPL_WRE ${CMAKE_CURRENT_SOURCE_DIR} ${FILES}"
    VERBATIM
    DEPENDS ${ARGN} ${PROJECT_SOURCE_DIR}/etc/scripts/tolua_generate.sh)
endfunction()

function(generate_interface_from_xml interface)
  get_filename_component(if_name ${interface} NAME_WLE)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.tolua
           ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.h
           ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp
    COMMAND ffifacegen -d ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/${interface}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${interface})
  include(lua)
  generate_tolua(${if_name} ${if_name}.tolua)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FAWKES_INTERFACE_DIR})
  add_library(${if_name} SHARED ${if_name}_tolua.cpp
                                ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp)
  target_include_directories(${if_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)
  target_link_libraries(${if_name} fawkescore fawkesutils fawkesinterface)

  # target_link_libraries(fawkesutils_tolua fawkescore fawkesutils)
endfunction()
