include(eigen3)
include(yamlcpp)

function(check_navgraph_deps NAVGRAPH_FOUND)
  if(YAMLCPP_FOUND AND EIGEN3_FOUND)
    set(NAVGRAPH_FOUND
        1
        PARENT_SCOPE)
  else()
    set(NAVGRAPH_FOUND
        0
        PARENT_SCOPE)
    if(NOT YAMLCPP_FOUND)
      message(WARNING "yaml-cpp[-devel] dependency missing")
    endif()
    if(NOT EIGEN3_FOUND)
      message(WARNING "eigen3 dependency missing")
    endif()
  endif()
endfunction()

function(depend_on_navgraph target)
  depend_on_yamlcpp(${target})
  depend_on_eigen3(${target})
  target_compile_options(${target} PUBLIC -DHAVE_YAMLCPP_0_5)
  target_link_libraries(${target} -lm)
  target_compile_features(${target} PUBLIC cxx_std_11)
endfunction()
