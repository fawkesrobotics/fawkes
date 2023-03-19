cmake_minimum_required(VERSION 3.5)
project(fawkes)
include(GNUInstallDirs)
find_package(PkgConfig REQUIRED)

# make local cmake modules available
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Debug")

set(FAWKES_INTERFACE_DIR ${CMAKE_BINARY_DIR}/src/libs/interfaces/)
set(FAWKES_PLUGIN_DIR ${PROJECT_SOURCE_DIR}/plugins/)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
# TODO: use compile_definitions instead
add_compile_options(
  -DBASEDIR=\"${PROJECT_SOURCE_DIR}\"
  -DFAWKES_BASEDIR=\"${PROJECT_SOURCE_DIR}\"
  -DBINDIR=\"${BINDIR}\"
  -DLIBDIR=\"${CMAKE_BINARY_DIR}/src/libs/\"
  -DPLUGINDIR=\"${FAWKES_PLUGIN_DIR}\"
  -DIFACEDIR=\"${FAWKES_INTERFACE_DIR}\"
  -DCONFDIR=\"${PROJECT_SOURCE_DIR}/cfg\"
  -DUSERDIR=\".fawkes\"
  -DLOGDIR=\"${PROJECT_SOURCE_DIR}/log\"
  -DRESDIR=\"${PROJECT_SOURCE_DIR}/res\"
  -DTMPDIR=\"/tmp\"
  -DSRCDIR=\"${PROJECT_SOURCE_DIR}/src\"
  -DBUILDTYPE=\"${BUILD_TYPE}\"
  -DSOEXT=\"so\")
# if(NOT CMAKE_CXX_STANDARD) set(CMAKE_CXX_STANDARD 17) endif()
add_subdirectory(src)
# add_subdirectory(src/libs) add_subdirectory(src/plugins)
# add_subdirectory(src/mainapp) add_subdirectory(src/tools)
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic) endif()
#
# collect the files for subdir1 include(src/libs//CMakeLists.txt) foreach(FILE
# ${FILES}) set(subdir1Files ${subdir1Files} subdir1/${FILE}) endforeach()
#
# collect the files for subdir2 include(subdir2/CMakeLists.txt) foreach(FILE
# ${FILES}) set(subdir2Files ${subdir2Files} subdir2/${FILE}) endforeach()
#
# add the source files to the executable add_executable(fawkes)
#
# ${subdir1Files} ${subdir2Files
