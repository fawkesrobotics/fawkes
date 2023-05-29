set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(GTest_FOUND)
  add_executable(test_skill_parser test_skill_parser.cpp)
  target_link_libraries(test_skill_parser m fawkes_execution_time_estimator
                        pthread GTest::gtest GTest::gtest_main)
  add_test(NAME Skillparser COMMAND test_skill_parser)
endif()
