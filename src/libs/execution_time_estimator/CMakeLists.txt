list(FIND CMAKE_CXX_COMPILE_FEATURES "cxx_std_20" _index)
if(${_index} GREATER -1)
  add_library(fawkes_execution_time_estimator SHARED
              execution_time_estimator.cpp)
  target_link_libraries(fawkes_execution_time_estimator fawkescore m
                        SkillerInterface)
  add_subdirectory(aspect)

  target_compile_features(fawkes_execution_time_estimator PUBLIC cxx_std_20)
else()
  message("Omitting execution_time_estimator (c++ 20 required)")
endif()
