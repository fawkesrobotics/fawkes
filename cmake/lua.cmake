# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

pkg_check_modules(LUA REQUIRED lua-5.1)

find_program(TOLUAPP tolua++ REQUIRED)

# Links target against lua and sets the necessary compile options
function(depend_on_lua target)
  set(exec_luadir ${PROJECT_SOURCE_DIR}/src/lua)
  set(exec_lualibdir ${PROJECT_SOURCE_DIR}/lib/lua)
  target_compile_options(
    ${target} PUBLIC ${LUA_CFLAGS} -DHAVE_LUA -DLUADIR=\"${exec_luadir}\"
                     -DLUALIBDIR=\"${exec_lualibdir}\" -Wno-unused-function)
  target_link_libraries(${target} ${LUA_LDFLAGS} tolua++)
  # TODO Secondexpansion stuff from lua.mk
endfunction()

# Generates the tolua bindings for a target
function(generate_tolua target)
  string(REPLACE ";" " " FILES "${ARGN}")
  add_custom_command(
    OUTPUT ${interface_prefix}${target}_tolua.pkg
           ${interface_prefix}${target}_tolua.cpp
    COMMAND
      /bin/sh -c
      "${FAWKES_CORE_DIR}/etc/scripts/./tolua_generate.sh \
${CMAKE_CURRENT_BINARY_DIR} ${interface_prefix}${target} \
${PROJECT_SOURCE_DIR}/src/lua/fawkes/toluaext.lua \
${PROJECT_SOURCE_DIR}/doc/headers/lichead_c.GPL_WRE \
${CMAKE_CURRENT_SOURCE_DIR} ${FILES}"
    VERBATIM
    DEPENDS ${ARGN} ${FAWKES_CORE_DIR}/etc/scripts/tolua_generate.sh
    COMMENT "Generate tolua bindings for ${target}")
endfunction()

# Links a target against a lua lib
function(generate_lua_lib target sources link_libs)
  generate_tolua(${target} "${sources}")
  add_library(${target}_tolua SHARED ${target}_tolua.cpp)
  set_target_properties(
    ${target}_tolua PROPERTIES PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${FAWKES_LUA_LIB_DIR})
  depend_on_lua(${target}_tolua)
  target_link_libraries(${target}_tolua "${link_libs}")
  set_target_properties(${target}_tolua PROPERTIES OUTPUT_NAME ${target})
endfunction()

# Converts interface xml to tolua bindings
function(generate_interface_from_xml interface)
  get_filename_component(if_name ${interface} NAME_WLE)
  set(interface_prefix interfaces_)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.tolua
           ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.h
           ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp
    COMMAND ffifacegen -d ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/${interface}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${interface}
    COMMENT "Generate c++ code and tolua bindings for ${if_name}")
  generate_tolua(${if_name} ${if_name}.tolua)
  add_library(${if_name}_tolua SHARED ${interface_prefix}${if_name}_tolua.cpp)
  depend_on_lua(${if_name}_tolua)
  set_target_properties(
    ${if_name}_tolua
    PROPERTIES PREFIX ""
               OUTPUT_NAME ${if_name}
               LIBRARY_OUTPUT_DIRECTORY ${FAWKES_LUA_INTERFACE_DIR})

  target_link_libraries(${if_name}_tolua fawkescore fawkesutils fawkesinterface
                        ${if_name})
  add_library(${if_name} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp)
  set_target_properties(${if_name} PROPERTIES
                          LIBRARY_OUTPUT_DIRECTORY ${FAWKES_INTERFACE_DIR})
  target_include_directories(${if_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)
  target_link_libraries(${if_name} fawkescore fawkesutils fawkesinterface)

  # target_link_libraries(fawkesutils_tolua fawkescore fawkesutils)
endfunction()
