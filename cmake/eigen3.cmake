pkg_check_modules(EIGEN3 eigen3)

function(depend_on_eigen3 target)
  target_compile_options(
    ${target} PUBLIC ${EIGEN3_CFLAGS} -DHAVE_EIGEN3 -DEIGEN_USE_NEW_STDVECTOR
                     -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
  target_link_libraries(${target} ${EIGEN3_LDFLAGS} tolua++)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${target} PRIVATE -Wno-deprecated-register)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${target} PRIVATE -Wno-deprecated-declarations)
  endif()
endfunction()
