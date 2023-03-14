pkg_check_modules(roscpp roscpp)

function(depend_on_ros target)
  target_compile_options(${target} PUBLIC -DHAVE_ROS)
  target_compile_options(${target} PRIVATE ${roscpp_CFLAGS}
                                           -DBOOST_BIND_GLOBAL_PLACEHOLDERS)
  target_link_libraries(${target} ${roscpp_LDFLAGS})
endfunction()
