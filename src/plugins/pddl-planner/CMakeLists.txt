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

set(PLUGIN_pddl-planner
    ON
    CACHE BOOL "Build pddl-planner plugin")

add_subdirectory(interfaces)

if(PLUGIN_pddl-planner)
  add_library(pddl-planner MODULE pddl-planner_plugin.cpp
                                  pddl-planner_thread.cpp)
  depend_on_mongodb(pddl-planner)
  depend_on_pkgconfig_libs(pddl-planner libctemplate)
  depend_on_boost_libs(pddl-planner "system;filesystem")

  target_link_libraries(
    pddl-planner
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fawkesrobotmemory
    PddlPlannerInterface)

  target_compile_features(pddl-planner PUBLIC cxx_std_17)
else()
  plugin_disabled_message("pddl-planner")
endif()
