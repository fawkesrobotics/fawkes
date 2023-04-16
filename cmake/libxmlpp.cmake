pkg_check_modules(libxml libxml++-2.6)
remember_dependency(libxml)

pkg_check_modules(glibmm-2.4 glibmm-2.4>=2.46)
remember_dependency(glibmm-2.4)

function(depend_on_libxml target)
  target_compile_definitions(${target} PRIVATE HAVE_LIBXMLPP)
  if(libxml_FOUND)

  endif()
  depend_on_pkgconfig_libs(${target} libxml)
  if(glibmm-2.4_FOUND)
    target_compile_options(${target} PRIVATE -Wno-deprecated-declarations)
    target_compile_features(${target} PRIVATE cxx_std_11)
  endif()
endfunction()
