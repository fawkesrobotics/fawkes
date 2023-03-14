if(Protobuf_FOUND AND ProtobufComm_FOUND)
  add_library(fawkesprotoboard SHARED protobuf_thread.cpp
                                      blackboard_manager.cpp protobuf_to_bb.cpp)
  depend_on_protobuf(fawkesprotoboard)
  depend_on_protobuf_comm(fawkesprotoboard)
  target_link_libraries(fawkesprotoboard fawkescore fawkesaspects
                        fawkesblackboard ProtobufPeerInterface)
endif()
