add_library(fawkes_execution_time_estimator SHARED execution_time_estimator.cpp)
target_link_libraries(fawkes_execution_time_estimator fawkescore m
                      SkillerInterface)
add_subdirectory(aspect)

target_compile_features(fawkes_execution_time_estimator PUBLIC cxx_std_20)
