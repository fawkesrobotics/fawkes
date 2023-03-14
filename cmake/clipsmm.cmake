pkg_check_modules(CLIPSMM clipsmm-1.0)
# list(REMOVE_ITEM CLIPSMM_CFLAGS -std=c++0x) pkg_check_modules(GLIBMM-2.4
# glibmm-2.4>=2.46)

function(depend_on_clipsmm target)
  # if(NOT GLIBMM-2-4_FOUND) target_compile_options(${target} PUBLIC
  # -Wno-deprecated-declarations -Wno-deprecated) endif()
  target_compile_options(${target} PUBLIC -DHAVE_CLIPS ${CLIPSMM_CFLAGS})
  target_link_libraries(${target} ${CLIPSMM_LDFLAGS})
  target_compile_features(${target} PUBLIC cxx_std_14)
endfunction()
