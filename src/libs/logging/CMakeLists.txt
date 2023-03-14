include_directories(libs)
add_library(
  fawkeslogging SHARED
  liblogger.cpp
  factory.cpp
  console.cpp
  cache.cpp
  file.cpp
  multi.cpp
  syslog.cpp
  component.cpp
  logger.cpp
  fd_redirect.cpp)
# TODO BOOST
find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
include(lua)
generate_tolua(fawkeslogging component.tolua)
add_library(fawkeslogging_tolua SHARED fawkeslogging_tolua.cpp)
depend_on_lua(fawkeslogging_tolua)

target_link_libraries(fawkeslogging pthread stdc++ fawkescore
                      ${Boost_SYSTEM_LIBRARY})
target_include_directories(fawkesutils PRIVATE logging)
