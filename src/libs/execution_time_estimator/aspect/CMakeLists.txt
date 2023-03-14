add_library(fawkes_execution_time_estimator_aspect SHARED
            execution_time_estimator.cpp execution_time_estimator_inifin.cpp)
target_link_libraries(fawkes_execution_time_estimator_aspect fawkescore
                      fawkesaspects fawkes_execution_time_estimator)
target_compile_features(fawkes_execution_time_estimator_aspect
                        PUBLIC cxx_std_20)
