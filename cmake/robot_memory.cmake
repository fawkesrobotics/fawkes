include(tf)
find_package(Boost)
pkg_check_modules(mongocxx libmongocxx>3)
if(mongocxx_FOUND
   AND Boost_FOUND
   AND TF_FOUND)
  set(robot_memory_FOUND 1)
else()
  set(robot_memory_WARNING "robot-memory:")
  if(NOT mongocxx_FOUND)
    list(APPEND robot_memory_WARNING
         " momgo-cxx-driver[-devel] >=3 dependency mssing")
  endif()
  if(NOT Boost_FOUND)
    list(APPEND robot_memory_WARNING
         " boost bind and/or function dependency mssing")
  endif()
  if(NOT TF_FOUND)
    list(APPEND robot_memory_WARNING " bullet[-devel] dependency mssing")
  endif()
  message(WARNING "${robot_memomry_WARNING}")
endif()

function(depend_on_mongodb target)
  depend_on_tf(${target})
  target_link_libraries(${target} ${mongocxx_LDFLAGS})
  target_compile_options(${target} PUBLIC ${mongocxx_CFLAGS} -Wno-deprecated)
endfunction()

function(depend_on_robot_memory target)
  depend_on_tf(${target})
  depend_on_mongodb(${target})
  target_include_directories(${target} PUBLIC ${Boost_INCLUDE_DIRS})
endfunction()
