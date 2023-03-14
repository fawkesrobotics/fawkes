add_executable(fawkes main.cpp)
target_link_libraries(
  fawkes
  stdc++
  fawkescore
  fawkesutils
  fawkesblackboard
  fawkesnetcomm
  fawkesconfig
  fawkesplugin
  fawkesaspects
  fawkesbaseapp)
target_include_directories(fawkescore PRIVATE core)
target_compile_options(fawkescore PRIVATE -DHAVE_EXECINFO)
