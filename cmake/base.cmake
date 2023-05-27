# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

include(GNUInstallDirs)
find_package(PkgConfig REQUIRED)

# make local cmake modules available
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${FAWKES_CORE_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Debug")

# TODO: .ts generation of rest-api plugins (see old Makefiles in rest-api dirs)
# This is not urgent, as the generated stuff is already in git this was handled
# through rest-api.mk add_custom_target(restapi-generate echo
# "generate-restapi") set(WEBVIEW_FRONTEND_SRCDIR
# ${PROJECT_SOURCE_DIR}/src/plugins/webview/frontend/src)

set(FAWKES_PLUGIN_DIR ${PROJECT_SOURCE_DIR}/plugins/)
set(FAWKES_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(FAWKES_RES_DIR ${FAWKES_CORE_DIR}/res)
set(FAWKES_LUA_LIB_DIR ${FAWKES_LIB_DIR}/lua)
set(FAWKES_INTERFACE_DIR ${FAWKES_LIB_DIR}/interfaces/)
set(FAWKES_LUA_INTERFACE_DIR ${FAWKES_LUA_LIB_DIR}/interfaces/)
set(FAWKES_EXECUTABLE_DIR ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${FAWKES_PLUGIN_DIR})
file(MAKE_DIRECTORY ${FAWKES_RES_DIR})
file(MAKE_DIRECTORY ${FAWKES_LIB_DIR})

file(MAKE_DIRECTORY ${FAWKES_LUA_LIB_DIR})
file(MAKE_DIRECTORY ${FAWKES_LUA_INTERFACE_DIR})
file(MAKE_DIRECTORY ${FAWKES_EXECUTABLE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${FAWKES_EXECUTABLE_DIR})
include_directories(${FAWKES_LIB_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${FAWKES_LIB_DIR})
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

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
include(utils)
include(dependencies)
include(libcrypto)
include(libxmlpp)
include(laser)
include(ros)
include(lua)
include(yamlcpp)
include(eigen3)
include(pcl)
include(tf)
include(gui)
include(navgraph)
include(cgal_libs)
include(webview)
include(fvconf)
include(kdl_parser)
include(clipsmm)
include(protobuf)
include(robot_memory)
