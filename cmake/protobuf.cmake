find_package(Protobuf)

function(depend_on_protobuf target)
  target_link_libraries(${target} ${Protobuf_LIBRARIES})
  target_include_directories(${target} PUBLIC ${Protobuf_INCLUDE_DIRS})
endfunction()

pkg_check_modules(PROTOBUF_COMM protobuf_comm)
function(depend_on_protobuf_comm target)
  target_compile_options(${target} PUBLIC ${PROTOBUF_COMM_CFLAGS})
  target_link_libraries(${target} ${PROTOBUF_COMM_LDFLAGS})
endfunction()
