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

function(depend_on_clipsmm target)
  depend_on_pkgconfig_libs(${target} clipsmm-1.0)
  target_compile_options(${target} PUBLIC -DHAVE_CLIPS ${CLIPSMM_CFLAGS})
  target_link_libraries(${target} ${CLIPSMM_LDFLAGS})
  target_compile_features(${target} PUBLIC cxx_std_14)
endfunction()
