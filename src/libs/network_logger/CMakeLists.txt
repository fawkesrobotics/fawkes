add_library(fawkesnetwork_logger SHARED network_logger.cpp)
target_link_libraries(fawkesnetwork_logger stdc++ pthread fawkescore
                      fawkeslogging fawkesnetcomm)
