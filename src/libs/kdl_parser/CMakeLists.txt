set(KDL_FOUND 0)
check_kdl_parser_deps(KDL_FOUND)

if(KDL_FOUND)
  add_library(
    fawkeskdl_parser SHARED
    # tests/test_kdlparser.cpp
    model.cpp kdl_parser.cpp)
  target_link_libraries(fawkeskdl_parser fawkescore)
  depend_on_kdl_parser(fawkeskdl_parser)
  depend_on_eigen3(fawkeskdl_parser)
  target_compile_features(fawkeskdl_parser PUBLIC cxx_std_11)
endif()
