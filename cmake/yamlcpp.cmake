pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
pkg_check_modules(YAMLCPP-0.5.3 yaml-cpp>=0.5.3)

function(depend_on_yamlcpp target)
  target_compile_options(${target} PUBLIC -DHAVE_YAMLCPP ${YAMLCPP_CFLAGS})
  target_link_libraries(${target} ${YAMLCPP_LDFLAGS})
  target_compile_options(${target} PUBLIC -DHAVE_YAMLCPP_NODE_MARK)
endfunction()
