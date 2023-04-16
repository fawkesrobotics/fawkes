pkg_check_modules(libmongocxx libmongocxx>3)
remember_dependency(libmongocxx)

function(depend_on_mongodb target)
  depend_on_tf(${target})
  depend_on_pkgconfig_libs(${target} libmongocxx)
  target_compile_options(${target} PUBLIC -Wno-deprecated)
endfunction()

function(depend_on_robot_memory target)
  depend_on_mongodb(${target})
endfunction()
