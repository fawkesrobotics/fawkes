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

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(COLOR_RESET "${Esc}[m")
  set(COLOR_BOLD "${Esc}[1m")
  set(RED "${Esc}[31m")
  set(GREEN "${Esc}[32m")
  set(YELLOW "${Esc}[33m")
  set(BLUE "${Esc}[34m")
  set(MAGENTA "${Esc}[35m")
  set(CYAN "${Esc}[36m")
  set(WHITE "${Esc}[37m")
  set(BOLD_RED "${Esc}[1;31m")
  set(BOLD_GREEN "${Esc}[1;32m")
  set(BOLD_YELLOW "${Esc}[1;33m")
  set(BOLD_BLUE "${Esc}[1;34m")
  set(BOLD_MAGENTA "${Esc}[1;35m")
  set(BOLD_CYAN "${Esc}[1;36m")
  set(BOLD_WHITE "${Esc}[1;37m")
endif()

function(optional_depend_on_boost_libs target libs success)
  set(${success}
      1
      PARENT_SCOPE)
  foreach(lib ${libs})
    string(TOUPPER ${lib} COMPONENT)
    set(tmp_list)
    list(APPEND tmp_list ${_FAWKES_DEPENDENCIES_CHECKED})
    if(NOT "boost_${lib}" IN_LIST tmp_list)
      find_package(Boost COMPONENTS ${lib})
      remember_dependency(boost_${lib})
    endif()
    if(Boost_${COMPONENT}_FOUND)
      target_link_libraries(${target} ${Boost_${COMPONENT}_LIBRARIES})
    else()
      set(${success}
          0
          PARENT_SCOPE)
    endif()
  endforeach()
  target_include_directories(${target} PUBLIC ${Boost_INCLUDE_DIR})
endfunction()

function(disable_target target)
  set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                             EXCLUDE_FROM_DEFAULT_BUILD 1)
endfunction()

function(build_depends_on target other_target)
  if($<TARGET_PROPERTY:${other_target},EXCLUDE_FROM_ALL>)
    disable_target(${target})
    build_skipped_message(${target} "target ${other_target}")

  endif()
endfunction()

function(depend_on_boost_libs target libs)
  foreach(lib ${libs})
    string(TOUPPER ${lib} COMPONENT)
    set(tmp_list)
    list(APPEND tmp_list ${_FAWKES_DEPENDENCIES_CHECKED})
    if(NOT "boost_${lib}" IN_LIST tmp_list)
      find_package(Boost COMPONENTS ${lib})
      remember_dependency(boost_${lib})
    endif()
    if(Boost_${COMPONENT}_FOUND)
      target_link_libraries(${target} ${Boost_${COMPONENT}_LIBRARIES})
    else()
      set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                                 EXCLUDE_FROM_DEFAULT_BUILD 1)
      target_skipped_message(${target} ${lib})
    endif()
  endforeach()
  target_include_directories(${target} PUBLIC ${Boost_INCLUDE_DIR})
endfunction()

function(depend_on_pkgconfig_libs target libs)
  foreach(lib ${libs})
    set(tmp_list)
    list(APPEND tmp_list ${_FAWKES_DEPENDENCIES_CHECKED})
    if(NOT ${lib} IN_LIST tmp_list)
      pkg_check_modules(${lib} QUIET ${lib})
      remember_dependency(${lib})
    endif()
    if(${lib}_FOUND)
      target_link_libraries(${target} ${${lib}_LDFLAGS})
      target_compile_options(${target} PUBLIC ${${lib}_CFLAGS})
    else()
      set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                                 EXCLUDE_FROM_DEFAULT_BUILD 1)
      target_skipped_message(${target} ${lib})
    endif()
  endforeach()
endfunction()

function(optional_depend_on_pkgconfig_libs target libs success)
  set(${success}
      1
      PARENT_SCOPE)
  set(tmp_list)
  list(APPEND tmp_list ${_FAWKES_DEPENDENCIES_CHECKED})
  foreach(lib ${libs})
    if(NOT ${lib} IN_LIST tmp_list)
      pkg_check_modules(${lib} QUIET ${lib})
      remember_dependency(${lib})
    endif()
    if(${lib}_FOUND)
      target_link_libraries(${target} ${${lib}_LDFLAGS})
      target_compile_options(${target} PUBLIC ${${lib}_CFLAGS})
    else()
      set(${success}
          0
          PARENT_SCOPE)
    endif()
  endforeach()
endfunction()

function(depend_on_find_package_libs target libs)
  foreach(lib ${libs})
    set(TMP_LIST)
    list(APPEND TMP_LIST ${FAWKES_DEPENDENCIES_CHECKED})
    if(NOT ${lib} IN_LIST TMP_LIST)
      find_package(${lib})
      remember_dependency(${lib})
    endif()
    if(${lib}_FOUND)
      set(${lib}_FOUND
          ${${lib}_FOUND}
          CACHE BOOL "")
      target_link_libraries(${target} ${${lib}_LINK_LIBRARIES})
      set(${lib}_INCLUDE_DIRS
          ${${lib}_INCLUDE_DIRS}
          CACHE STRING "")
      set(${lib}_CFLAGS
          ${${lib}_CFLAGS}
          CACHE STRING "")
      target_include_directories(${target} PUBLIC ${${lib}_INCLUDE_DIRS})
      target_compile_options(${target} PUBLIC ${${lib}_CFLAGS})
    else()
      set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                                 EXCLUDE_FROM_DEFAULT_BUILD 1)
      target_skipped_message(${target} ${lib})
    endif()
  endforeach()
endfunction()

macro(SET_COMMON_PROPERTIES_OF_TARGETS_RECUCURSIVE targets dir)
  get_property(
    subdirectories
    DIRECTORY ${dir}
    PROPERTY SUBDIRECTORIES)
  foreach(subdir ${subdirectories})
    set_common_properties_of_targets_recucursive(${targets} ${subdir})
  endforeach()

  get_property(
    current_targets
    DIRECTORY ${dir}
    PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND ${targets} ${current_targets})
  foreach(target ${current_targets})
    get_target_property(type ${target} TYPE)
    if(${type} MATCHES "MODULE_LIBRARY"
       OR ${type} MATCHES "SHARED_LIBRARY"
       OR ${type} MATCHES "EXECUTABLE")
      target_compile_definitions(
        ${target}
        PRIVATE SRCDIR=\"${dir}\"
                BASEDIR=\"${PROJECT_SOURCE_DIR}\"
                FAWKES_BASEDIR=\"${PROJECT_SOURCE_DIR}\"
                BINDIR=\"${BINDIR}\"
                LIBDIR=\"${CMAKE_BINARY_DIR}/src/libs/\"
                PLUGINDIR=\"${FAWKES_PLUGIN_DIR}\"
                IFACEDIR=\"${FAWKES_INTERFACE_DIR}\"
                CONFDIR=\"${PROJECT_SOURCE_DIR}/cfg\"
                USERDIR=\".fawkes\"
                LOGDIR=\"${PROJECT_SOURCE_DIR}/log\"
                RESDIR=\"${PROJECT_SOURCE_DIR}/res\"
                TMPDIR=\"/tmp\"
                BUILDTYPE=\"${BUILD_TYPE}\"
                SOEXT=\"so\")
    endif()
  endforeach()
endmacro()

function(set_common_properties_of_targets var)
  set(targets)
  set_common_properties_of_targets_recucursive(targets
                                               ${CMAKE_CURRENT_SOURCE_DIR})
  set(${var}
      ${targets}
      PARENT_SCOPE)
endfunction()

function(remember_dependency dep)
  set(_FAWKES_DEPENDENCIES_CHECKED
      "${dep};${_FAWKES_DEPENDENCIES_CHECKED}"
      CACHE INTERNAL "")
endfunction()

function(set_output_directory_plugins)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
      ${PROJECT_SOURCE_DIR}/plugins
      PARENT_SCOPE)
endfunction()

function(set_output_directory_libs)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/src/libs/
      PARENT_SCOPE)
endfunction()

function(unset_output_directory)
  unset(CMAKE_LIBRARY_OUTPUT_DIRECTORY PARENT_SCOPE)
endfunction()

function(plugin_disabled_message plugin)
  message(
    STATUS "${BOLD_GREEN}Skip building disabled ${plugin} plugin${COLOR_RESET}")
endfunction()

function(build_skipped_message component reason)
  message(
    STATUS
      "${BOLD_WHITE}Omitting ${component} (${reason} required)${COLOR_RESET}")
endfunction()

function(target_skipped_message component reason)
  message(
    STATUS
      "${BOLD_MAGENTA}Omitting ${component} (${reason} required)${COLOR_RESET}")
endfunction()

function(executable_disabled_message exe)
  message(
    STATUS "${BOLD_BLUE}Skip building disabled ${exe} executable${ColorReset}")
endfunction()

function(cmake_not_implemented target)
  add_custom_target(
    ${target} ALL
    ${CMAKE_COMMAND} -E cmake_echo_color --yellow --bold
    "target ${target} cannot be built by cmake yet"
    COMMAND exit 1
    COMMENT
      "Dummy target to indicate components that are not built through cmake yet"
    VERBATIM)
endfunction()

function(create_symlink source destination)
  get_filename_component(target ${source} NAME)
  create_symlink_custom_target(${target} ${source} ${destination}/${target})
endfunction()

function(create_symlink_custom_target target source destination)
  add_custom_target(
    ${target} ALL
    COMMAND ${CMAKE_COMMAND} -E true
    COMMENT "Target ${target} for symlink")
  get_filename_component(source_name ${source} NAME)

  add_custom_command(
    TARGET ${target}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${CMAKE_CURRENT_SOURCE_DIR}/${source} ${destination}
    COMMENT
      "${BOLD_BLUE}Created symlink: ${CMAKE_CURRENT_SOURCE_DIR}/${target} -> ${destination}${COLOR_RESET}"
  )
endfunction()

list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_11" _index)
if(${_index} GREATER -1)
  set(CPP_11_FOUND 1)
endif()
list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_13" _index)
if(${_index} GREATER -1)
  set(CPP_14_FOUND 1)
endif()
list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_17" _index)
if(${_index} GREATER -1)
  set(CPP_17_FOUND 1)
endif()
list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_20" _index)
if(${_index} GREATER -1)
  set(CPP_20_FOUND 1)
endif()

function(depend_on_cpp_version target version)
  if(${CPP_${version}_FOUND})
    target_compile_features(${target} PRIVATE cxx_std_${version})
  else()
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                               EXCLUDE_FROM_DEFAULT_BUILD 1)
    target_skipped_message(${target} "C++ ${version}")
  endif()
endfunction()
