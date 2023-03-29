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

include(tf)
if(TF_FOUND)
  # TODO This must be PUBLIC, should everything else be aswell?
  target_compile_options(fawkestf PUBLIC ${TF_CFLAGS})
  # we're only interested in the math part
  target_link_libraries(fawkestf ${TF_LDFLAGS})
  target_compile_features(fawkestf PUBLIC cxx_std_11)

  target_compile_options(fawkestf PUBLIC ${TF_CFLAGS})
  set(LUA_SOURCES tf.tolua)
  set(LUA_LINK_LIBS fawkescore fawkesutils fawkestf ${TF_LDFLAGS})
  generate_lua_lib(fawkestf "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
else()
  message(WARNING "Omitting transforms library (bullet[-devel] not installed)")
endif()
