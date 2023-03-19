include(ros)
include(eigen3)

set(PCL_SUFFIX "")
pkg_check_modules(PCL pcl_common)
if(NOT PCL_FOUND)
  message("Check for plc_common with explicit version suffix")
  execute_process(
    COMMAND pkg-config --list-all
    COMMAND grep pcl_common
    COMMAND awk "{ print $1 }"
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE PCL_OUTPUT)
  pkg_check_modules(PCL ${PCL_OUTPUT})
  string(REGEX REPLACE "^pcl_common" "" PCL_SUFFIX ${PCL_OUTPUT})
endif()
if(EXISTS "${CMAKE_SYSROOT}/usr/include/vtk/vtkVersion.h")
  set(CFLAGS_VTK -I/usr/include/vtk)
endif()
function(depend_on_pcl target)
  depend_on_eigen3(${target})
  target_compile_options(${target} PUBLIC -DHAVE_PCL)
  target_compile_options(
    ${target}
    PRIVATE ${PCL_CFLAGS} -Wno-unknown-pragmas -Wno-deprecated-declarations
            -Wno-overloaded-virtual -Wno-array-bounds)
  target_link_libraries(${target} ${PCL_LDFLAGS})
endfunction()

function(check_deps_pcl_extra_libs libs)
  set(PCL_WARNING)
  foreach(pcl_lib ${libs})
    if(NOT PCL_${pcl_lib}_FOUND)
      pkg_check_modules(PCL_${pcl_lib} pcl_${pcl_lib}${PCL_SUFFIX})
      if(NOT PCL_${pcl_lib}_FOUND)
        list(APPEND PCL_WARNING "${pcl_lib} dependency missing")
      endif()
    endif()
  endforeach()
  set(PCL_WARNING
      ${PCL_WARNING}
      PARENT_SCOPE)
endfunction()

function(depend_on_pcl_extra_libs target libs)
  foreach(pcl_lib ${libs})
    if(PCL_${pcl_lib}_FOUND)
      target_link_libraries(${target} ${PCL_${pcl_lib}_LDFLAGS})
      target_compile_options(${target} PUBLIC ${PCL_${pcl_lib}_CFLAGS})
    endif()
  endforeach()
endfunction()

# TODO link against ros std_msgs!?
