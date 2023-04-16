add_library(
  fawkestf SHARED
  time_cache.cpp
  # qa/qa_tf_transformer.cpp
  static_cache.cpp
  transform_listener.cpp
  transformer.cpp
  exceptions.cpp
  buffer_core.cpp
  fawkestf_tolua.cpp
  transform_publisher.cpp
  types.h)

target_link_libraries(fawkestf fawkescore fawkesutils fawkesblackboard
                      fawkesinterface TransformInterface)

depend_on_tf(fawkestf)
target_compile_features(fawkestf PUBLIC cxx_std_11)

target_compile_definitions(
  fawkestf PUBLIC -DBT_INFINITY -DBT_USE_DOUBLE_PRECISION
                  -DBT_EULER_DEFAULT_ZYX)

set(LUA_SOURCES tf.tolua)
set(LUA_LINK_LIBS fawkescore fawkesutils fawkestf)
generate_lua_lib(fawkestf "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
