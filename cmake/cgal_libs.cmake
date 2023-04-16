function(depend_on_cgal_libs target)
  if(NOT CGAL IN_LIST FAWKES_DEPENDENCIES_CHECKED)
    find_package(CGAL)
    remember_dependency(CGAL)
  endif()

  depend_on_pkgconfig_libs(${target} "gmp;mpfr")
  depend_on_boost_libs(${target} "system;thread")
  target_include_directories(${target} PUBLIC ${Boost_INCLUDE_DIR})
  target_link_libraries(${target} ${CGAL_LDLAGS} ${GMP_LDFLAGS} ${MPFR_LDFLAGS}
                        -lm)
  target_compile_options(
    ${target}
    PUBLIC ${CGAL_LDLAGS}
           ${GMP_LDFLAGS}
           ${MPFR_LDFLAGS}
           -DHAVE_CGAL
           -Wno-depcrecated-register
           -DCGAL_DISABLE_ROUNDING_MATH_CHECK
           -frounding-math)
  # TODO: clang flags: -Wno-unused-local-typedef
endfunction()
