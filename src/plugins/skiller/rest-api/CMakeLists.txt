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

set(PLUGIN_skiller-rest-api
    ON
    CACHE BOOL ON)
if(PLUGIN_skiller-rest-api)
  add_library(
    skiller-rest-api MODULE
    model/SkillCall.cpp model/Skill.cpp model/SkillInfo.cpp
    skiller-rest-api-plugin.cpp skiller-rest-api.cpp)

  depend_on_webview(skiller-rest-api)

  target_link_libraries(
    skiller-rest-api
    fawkescore
    fawkesutils
    fawkesaspects
    fawkeswebview
    fawkesblackboard
    SkillerDebugInterface
    SkillerInterface)
  target_compile_features(skiller-rest-api PRIVATE cxx_std_17)
else()
  plugin_disabled_message(skiller-rest-api)
endif()
