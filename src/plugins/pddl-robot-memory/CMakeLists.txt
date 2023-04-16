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

set(PLUGIN_pddl-robot-memory
    ON
    CACHE BOOL ON)

find_package(Boost COMPONENTS system filesystem)

add_subdirectory(interfaces)

if(PLUGIN_pddl-robot-memory)
  add_library(pddl-robot-memory MODULE pddl_robot_memory_plugin.cpp
                                       pddl_robot_memory_thread.cpp)
  depend_on_robot_memory(pddl-robot-memory)
  depend_on_pkgconfig_libs(pddl-robot-memory libctemplate)
  target_link_libraries(
    pddl-robot-memory
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fawkesrobotmemory
    PddlGenInterface)

  target_compile_features(pddl-robot-memory PUBLIC cxx_std_17)

else()
  plugin_disabled_message("pddl-robot-memory")
endif()
