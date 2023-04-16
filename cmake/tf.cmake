set(tf_deps "bullet")

function(depend_on_tf target)
  set(TF_CFLAGS -DHAVE_TF -DBT_INFINITY -DBT_USE_DOUBLE_PRECISION
                -DB_EULER_DEFAULT_ZYX -O2)
  set(TF_LDFLAGS -lLinearMath -lm)
  target_link_libraries(${target} ${TF_LDFLAGS})
  target_compile_options(${target} PUBLIC ${TF_CFLAGS})
  depend_on_pkgconfig_libs(${target} bullet)
endfunction()
