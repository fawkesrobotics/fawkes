set(eigen3_deps "eigen3")

function(depend_on_eigen3 target)
  depend_on_pkgconfig_libs(${target} eigen3)
  target_compile_options(
    ${target} PUBLIC -DHAVE_EIGEN3 -DEIGEN_USE_NEW_STDVECTOR
                     -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
  target_link_libraries(${target} tolua++)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${target} PRIVATE -Wno-deprecated-register)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${target} PRIVATE -Wno-deprecated-declarations)
  endif()
endfunction()
