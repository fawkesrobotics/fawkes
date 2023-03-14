pkg_check_modules(bullet bullet)
if(bullet_FOUND)
  set(TF_FOUND 1)
  set(TF_CFLAGS -DHAVE_TF -DBT_INFINITY ${bullet_CFLAGS}
                -DBT_USE_DOUBLE_PRECISION -DB_EULER_DEFAULT_ZYX -O2)
  set(TF_LDFLAGS -lLinearMath -lm)
endif()

function(depend_on_tf target)
  target_link_libraries(${target} ${TF_LDFLAGS})
  target_compile_options(${target} PUBLIC ${TF_CFLAGS})
endfunction()
