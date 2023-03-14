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
