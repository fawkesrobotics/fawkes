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

  include(lua)
  generate_tolua(fawkestf tf.tolua)
  add_library(fawkestf_tolua SHARED fawkestf_tolua.cpp)
  depend_on_lua(fawkestf_tolua)
  target_compile_options(fawkestf PUBLIC ${TF_CFLAGS})
  target_link_libraries(fawkestf_tolua fawkescore fawkesutils fawkestf
                        ${TF_LDFLAGS})
else()
  message(WARNING "Omitting transforms library (bullet[-devel] not installed)")
endif()
