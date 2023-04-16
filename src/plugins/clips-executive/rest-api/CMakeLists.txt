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

set(PLUGIN_clips-executive-rest-api
    ON
    CACHE BOOL ON)
if(PLUGIN_clips-executive-rest-api)
  add_library(
    clips-executive-rest-api MODULE
    clips-executive-rest-api.cpp
    clips-executive-rest-api-plugin.cpp
    model/Plan.cpp
    model/DomainEffect.cpp
    model/Goal.cpp
    model/GroundedPDDLFormula.cpp
    model/DomainObjectType.cpp
    model/GroundedPDDLPredicate.cpp
    model/DomainFact.cpp
    model/GroundedFormula.cpp
    model/DomainPredicate.cpp
    model/PDDLPredicate.cpp
    model/PlanAction.cpp
    model/PDDLFormula.cpp
    model/DomainObject.cpp
    model/DomainOperator.cpp
    model/PDDLGrounding.cpp
    model/DomainOperatorParameter.cpp)

  depend_on_webview(clips-executive-rest-api)

  target_link_libraries(clips-executive-rest-api fawkescore fawkesutils
                        fawkesaspects fawkesclipsaspect fawkeswebview)
  target_compile_features(clips-executive-rest-api PRIVATE cxx_std_17)
else()
  plugin_disabled_message(clips-executive-rest-api)
endif()
