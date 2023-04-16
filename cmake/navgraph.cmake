include(eigen3)
include(yamlcpp)

set(navgraph_deps ${yamlcpp_deps};${eigen3_deps})

function(depend_on_navgraph target)
  depend_on_yamlcpp(${target})
  depend_on_eigen3(${target})
  target_compile_options(${target} PUBLIC -DHAVE_YAMLCPP_0_5)
  target_link_libraries(${target} -lm)
  target_compile_features(${target} PUBLIC cxx_std_11)
endfunction()
