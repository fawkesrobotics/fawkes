function(check_deps_extra_libs libs)
  set(EXTRA_LIBS_WARNING)
  foreach(lib ${libs})
    if(NOT ${lib}_FOUND)
      pkg_check_modules(${lib} ${lib})
      if(NOT ${lib}_FOUND)
        list(APPEND EXTRA_LIBS_WARNING "${lib} dependency missing")
      endif()
    endif()
  endforeach()
  set(EXTRA_LIBS_WARNING
      ${EXTRA_LIBS_WARNING}
      PARENT_SCOPE)
endfunction()

function(depend_on_extra_libs target libs)
  foreach(lib ${libs})
    if(${lib}_FOUND)
      target_link_libraries(${target} ${${lib}_LDFLAGS})
      target_compile_options(${target} PUBLIC ${${lib}_CFLAGS})
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
  message(STATUS "Skip building disabled ${plugin} plugin")
endfunction()

function(build_skipped_message component reason)
  message(WARNING "Omitting ${component} (${reason} required)")
endfunction()

function(executable_disabled_message exe)
  message(STATUS "Skip building disabled ${exe} executable")
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
