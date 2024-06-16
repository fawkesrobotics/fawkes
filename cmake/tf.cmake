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

set(TF_DEPS "bullet")
# Function: depend_on_tf
# Usage: depend_on_tf(TARGET_NAME)
# Adds the ros_tf dependency  to the target
function(depend_on_tf target)
  set(tf_cflags -DHAVE_TF -DBT_INFINITY -DBT_USE_DOUBLE_PRECISION
                -DB_EULER_DEFAULT_ZYX -O2)
  set(tf_ldflags -lLinearMath -lm)
  target_link_libraries(${target} ${tf_ldflags})
  target_compile_options(${target} PUBLIC ${tf_cflags})
  depend_on_pkgconfig_libs(${target} bullet)
endfunction()
