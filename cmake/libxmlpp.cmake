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

pkg_check_modules(libxml libxml++-2.6)
remember_dependency(libxml)

pkg_check_modules(glibmm-2.4 glibmm-2.4>=2.46)
remember_dependency(glibmm-2.4)

# Function: depend_on_libxml
# Usage: depend_on_libxml(TARGET_NAME)
#
# Adds the dependency for libxml to the target.
function(depend_on_libxml target)
  target_compile_definitions(${target} PRIVATE HAVE_LIBXMLPP)
  if(libxml_FOUND)

  endif()
  depend_on_pkgconfig_libs(${target} libxml)
  if(glibmm-2.4_FOUND)
    target_compile_options(${target} PRIVATE -Wno-deprecated-declarations)
    target_compile_features(${target} PRIVATE cxx_std_11)
  endif()
endfunction()
