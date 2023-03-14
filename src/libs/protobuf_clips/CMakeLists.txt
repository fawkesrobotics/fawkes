find_package(Boost COMPONENTS system)

if(Protobuf_FOUND
   AND CLIPSMM_FOUND
   AND PROTOBUF_COMM_FOUND
   AND Boost_SYSTEM_FOUND)
  add_library(fawkes_protobuf_clips SHARED communicator.cpp)

  target_link_libraries(fawkes_protobuf_clips stdc++ m fawkescore fawkeslogging
                        ${Boost_SYSTEM_LIBRARY})
  target_compile_options(fawkesbaseapp PUBLIC)
  depend_on_protobuf(fawkes_protobuf_clips)
  depend_on_protobuf_comm(fawkes_protobuf_clips)
  depend_on_clipsmm(fawkes_protobuf_clips)
else()
  message(WARNING "Omitting protobuf_clips library")
  if(NOT Protobuf_FOUND)
    message(WARNING "protobuf[-devel] dependency missing")
  endif()
  if(NOT PROTOBUF_COMM_FOUND)
    message(WARNING "protobuf_comm[-devel] dependency missing")
  endif()
  if(NOT CLIPSMM_FOUND)
    message(WARNING "clipsmm[-devel] dependency missing")
  endif()
  if(NOT Boost_SYSTEM_FOUND)
    message(WARNING "boost system dependency missing")
  endif()
endif()
