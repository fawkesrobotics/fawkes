include(ros)
include(eigen3)

set(PCL_SUFFIX "")
set(PCL_COMMON "pcl_common")
pkg_check_modules(PCL pcl_common)
remember_dependency(PCL)

if(NOT PCL_FOUND)
  message("Check for plc_common with explicit version suffix")
  execute_process(
    COMMAND pkg-config --list-all
    COMMAND grep pcl_common
    COMMAND awk "{ print $1 }"
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE PCL_COMMON)
  pkg_check_modules(PCL ${PCL_COMMON})
  string(REGEX REPLACE "^pcl_common" "" PCL_SUFFIX ${PCL_COMMON})
endif()
if(EXISTS "${CMAKE_SYSROOT}/usr/include/vtk/vtkVersion.h")
  set(CFLAGS_VTK -I/usr/include/vtk)
endif()
function(depend_on_pcl target)
  depend_on_pkgconfig_libs(${target} ${PCL_COMMON})
  depend_on_eigen3(${target})
  target_compile_options(${target} PUBLIC -DHAVE_PCL)
  target_compile_options(
    ${target} PRIVATE -Wno-unknown-pragmas -Wno-deprecated-declarations
                      -Wno-overloaded-virtual -Wno-array-bounds)
endfunction()

function(depend_on_pcl_extra_libs target libs)
  set(PCL_LIBS)
  foreach(pcl_lib ${libs})
    list(APPEND PCL_LIBS pcl_${pcl_lib}${PCL_SUFFIX})
  endforeach()
  depend_on_pkgconfig_libs(${target} ${PCL_LIBS})
endfunction()

# TODO link against ros std_msgs!?
