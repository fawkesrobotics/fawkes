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
  set(ColourReset "${Esc}[m")
  set(ColourBold "${Esc}[1m")
  set(Red "${Esc}[31m")
  set(Green "${Esc}[32m")
  set(Yellow "${Esc}[33m")
  set(Blue "${Esc}[34m")
  set(Magenta "${Esc}[35m")
  set(Cyan "${Esc}[36m")
  set(White "${Esc}[37m")
  set(BoldRed "${Esc}[1;31m")
  set(BoldGreen "${Esc}[1;32m")
  set(BoldYellow "${Esc}[1;33m")
  set(BoldBlue "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan "${Esc}[1;36m")
  set(BoldWhite "${Esc}[1;37m")
endif()

function(optional_depend_on_boost_libs target libs success)
  set(${success}
      1
      PARENT_SCOPE)
  foreach(lib ${libs})
    string(TOUPPER ${lib} COMPONENT)
    set(TMP_LIST)
    list(APPEND TMP_LIST ${FAWKES_DEPENDENCIES_CHECKED})
    if(NOT "boost_${lib}" IN_LIST TMP_LIST)
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

function(build_depends_on target other-target)
  if($<TARGET_PROPERTY:${other-target},EXCLUDE_FROM_ALL>)
    disable_target(${target})
    build_skipped_message(${target} "target ${other-target}")

  endif()
endfunction()

function(depend_on_boost_libs target libs)
  foreach(lib ${libs})
    string(TOUPPER ${lib} COMPONENT)
    set(TMP_LIST)
    list(APPEND TMP_LIST ${FAWKES_DEPENDENCIES_CHECKED})
    if(NOT "boost_${lib}" IN_LIST TMP_LIST)
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
    set(TMP_LIST)
    list(APPEND TMP_LIST ${FAWKES_DEPENDENCIES_CHECKED})
    if(NOT ${lib} IN_LIST TMP_LIST)
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
  set(TMP_LIST)
  list(APPEND TMP_LIST ${FAWKES_DEPENDENCIES_CHECKED})
  foreach(lib ${libs})
    if(NOT ${lib} IN_LIST TMP_LIST)
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

macro(set_common_properties_of_targets_recursive targets dir)
  get_property(
    subdirectories
    DIRECTORY ${dir}
    PROPERTY SUBDIRECTORIES)
  foreach(subdir ${subdirectories})
    set_common_properties_of_targets_recursive(${targets} ${subdir})
  endforeach()

  get_property(
    current_targets
    DIRECTORY ${dir}
    PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND ${targets} ${current_targets})
  foreach(target ${current_targets})
    if(NOT ${target} IN_LIST FAWKES_CUSTOM_TARGETS)
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
  set_common_properties_of_targets_recursive(targets
                                             ${CMAKE_CURRENT_SOURCE_DIR})
  set(${var}
      ${targets}
      PARENT_SCOPE)
endfunction()

function(remember_dependency dep)
  set(FAWKES_DEPENDENCIES_CHECKED
      "${dep};${FAWKES_DEPENDENCIES_CHECKED}"
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
    STATUS "${BoldGreen}Skip building disabled ${plugin} plugin${ColourReset}")
endfunction()

function(build_skipped_message component reason)
  message(
    STATUS
      "${BoldWhite}Omitting ${component} (${reason} required)${ColourReset}")
endfunction()

function(target_skipped_message component reason)
  message(
    STATUS
      "${BoldMagenta}Omitting ${component} (${reason} required)${ColourReset}")
endfunction()

function(executable_disabled_message exe)
  message(
    STATUS "${BoldBlue}Skip building disabled ${exe} executable${ColorReset}")
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
