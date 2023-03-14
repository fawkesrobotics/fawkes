find_package(Boost COMPONENTS system thread)
find_package(CGAL)
pkg_check_modules(GMP gmp)
pkg_check_modules(MPFR mpfr)

function(check_cgal_libs_deps CGAL_LIBS_FOUND)
  if(CGAL_FOUND
     AND GMP_FOUND
     AND MPFR_FOUND
     AND BOOST_FOUND)
    set(CGAL_LIBS_FOUND
        1
        PARENT_SCOPE)
  else()
    set(CGAL_LIBS_FOUND
        0
        PARENT_SCOPE)
    if(NOT CGAL_FOUND)
      message(WARNING "CGAL[-devel] dependency missing")
    endif()
    if(NOT GMP_FOUND)
      message(WARNING "gmp[-devel] dependency missing")
    endif()
    if(NOT MPFR_FOUND)
      message(WARNING "mpfr[-devel] dependency missing")
    endif()
    if(NOT BOOST_FOUND)
      message(WARNING "boost system or thread dependency missing")
    endif()
  endif()
endfunction()

function(depend_on_cgal_libs target)
  target_include_directories(${target} PUBLIC ${Boost_INCLUDE_DIR})
  target_link_libraries(${target} ${Boost_LIBRARIES} ${CGAL_LDLAGS}
                        ${GMP_LDFLAGS} ${MPFR_LDFLAGS} -lm)
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
