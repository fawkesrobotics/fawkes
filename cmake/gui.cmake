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

pkg_check_modules(GTKMM gtkmm-3.0>=3.0.0)
if(NOT GTKMM_FOUND)
  pkg_check_modules(GTKMM gtkmm-2.4>=2.12)
endif()
remember_dependency(GTKMM)

# Function: depend_on_gui
# Usage: depend_on_gui(TARGET_NAME)
#
# Adds the necessary dependencies for gui to the target.
function(depend_on_gui target)
  depend_on_pkgconfig_libs(${target} "cairomm-1.0;GTKMM;gthread-2.0")
  optional_depend_on_pkgconfig_libs(${target} giomm-2.4 giomm_dep_found)
  optional_depend_on_pkgconfig_libs(${target} gconfmm-2.6 gconfmm_dep_found)
  optional_depend_on_pkgconfig_libs(${target} glibmm-2.4 glibmm_dep_found)
  target_compile_options(
    ${target} PUBLIC -DHAVE_GTKMM -DGLIB_VERSION_MIN_REQUIRED=GLIB_VERSION_2_44
                     -DGLIB_VERSION_MAX_ALLOWED=GLIB_VERSION_2_60)
endfunction()
