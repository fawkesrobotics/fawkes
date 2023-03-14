include(ros)
include(eigen3)

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

# TODO link against ros std_msgs!?
