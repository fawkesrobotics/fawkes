set(BUILD_WITH_ROS_1
    ON
    CACHE BOOL "Build with ROS ")

if(BUILD_WITH_ROS_1)
  pkg_check_modules(roscpp roscpp)
  remember_dependency(roscpp)
  if(roscpp_FOUND)
    set(ROS_1_FOUND 1)
  endif()
endif()

function(depend_on_ros_libs target libs)
  if(BUILD_WITH_ROS_1 AND ROS_1_FOUND)
    depend_on_pkgconfig_libs(${target} ${libs})
  else()
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                               EXCLUDE_FROM_DEFAULT_BUILD 1)
    if(NOT BUILD_WITH_ROS_1)
      target_skipped_message(${target} "ROS 1 disabled (BUILD_WITH_ROS_1)")
    else()
      if(NOT ROS_1_FOUND)
        target_skipped_message(${target} "ros-roscpp[-devel]")
      endif()
    endif()
  endif()
endfunction()

function(depend_on_ros target)
  depend_on_ros_libs(${target} roscpp)
  target_compile_options(${target} PUBLIC -DHAVE_ROS)
  target_compile_options(${target} PRIVATE -DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endfunction()

function(optional_depend_on_ros_libs target libs success)
  if(BUILD_WITH_ROS_1 AND ROS_1_FOUND)
    optional_depend_on_pkgconfig_libs(${target} ${libs} success)
    if(success)
      set(${success}
          1
          PARENT_SCOPE)
    endif()
  endif()
endfunction()
