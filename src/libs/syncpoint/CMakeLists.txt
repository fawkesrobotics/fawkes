add_library(
  fawkessyncpoint SHARED syncpoint.cpp syncpoint_manager.cpp syncpoint_call.cpp
                         syncpoint_call_stats.cpp # tests/test_syncpoint.cpp
)
target_link_libraries(fawkessyncpoint fawkescore fawkesutils)
