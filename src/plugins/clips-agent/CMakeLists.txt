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

set(PLUGIN_clips-agent
    ON
    CACHE BOOL "Build clips-agent plugin")

if(PLUGIN_clips-agent)

  add_library(clips-agent MODULE clips_agent_plugin.cpp clips_agent_thread.cpp)

  depend_on_clipsmm(clips-agent)

  target_link_libraries(
    clips-agent
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    SkillerInterface
    SwitchInterface)

  # install(FILES clips/init.clp clips/skills.clp)

else()
  plugin_disabled_message(clips-agent)
endif()
