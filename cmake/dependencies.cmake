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

pkg_check_modules(yaml-cpp REQUIRED yaml-cpp>=0.5.3)
remember_dependency(yaml-cpp)

pkg_check_modules(libelf REQUIRED libelf)
remember_dependency(libelf)

find_package(Boost COMPONENTS system thread filesystem chrono)
if(Boost_SYSTEM_FOUND AND Boost_THREAD_FOUND)
  set(SICK_TIM55X_BOOST_DEPS_FOUND
      1
      CACHE BOOL "SICK_TIM55x boost dependencies")
  add_library(sick_tim55x INTERFACE IMPORTED)
  target_include_directories(sick_tim55x INTERFACE ${Boost_INCLUDE_DIRS})
  target_link_libraries(sick_tim55x INTERFACE -lpthread ${Boost_LIBRARIES})
  target_compile_definitions(sick_tim55x INTERFACE HAVE_SICK55X_BOOST)
  # target_compile_definitions do not work with imported targets
endif()

find_path(URG_ROOT urg/UrgCtrl.h PATH_SUFFIXES include)
if(URG_ROOT)
  add_library(liburg INTERFACE IMPORTED)
  target_include_directories(liburg INTERFACE ${URG_ROOT})
  target_link_directories(liburg INTERFACE ${URG_ROOT}/lib)
  target_link_libraries(
    liburg
    INTERFACE urg
              urg_connection
              urg_monitor
              urg_coordinate
              urg_geometry
              urg_system
              urg_common
              urg_connection_sdl)
  set(URG_C_DEFINITION HAVE_URG)
endif()

find_path(FLITE_ROOT flite/flite.h PATH_SUFFIXES include)
if(FLITE_ROOT)
  add_library(FLite INTERFACE IMPORTED)
  target_include_directories(FLite INTERFACE ${FLITE_ROOT})
  target_link_directories(FLite INTERFACE ${FLITE_ROOT}/lib)
  target_link_libraries(FLite INTERFACE -lflite_cmu_us_kal -lflite_usenglish
                                        -lflite_cmulex -lflite -lm)
endif()
