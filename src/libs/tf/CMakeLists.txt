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

add_library(
  fawkestf SHARED
  time_cache.cpp
  # qa/qa_tf_transformer.cpp
  static_cache.cpp
  transform_listener.cpp
  transformer.cpp
  exceptions.cpp
  buffer_core.cpp
  fawkestf_tolua.cpp
  transform_publisher.cpp
  types.h)

target_link_libraries(fawkestf fawkescore fawkesutils fawkesblackboard
                      fawkesinterface TransformInterface)

depend_on_tf(fawkestf)
target_compile_features(fawkestf PUBLIC cxx_std_11)

target_compile_definitions(
  fawkestf PUBLIC -DBT_INFINITY -DBT_USE_DOUBLE_PRECISION
                  -DBT_EULER_DEFAULT_ZYX)

set(LUA_SOURCES tf.tolua)
set(LUA_LINK_LIBS fawkescore fawkesutils fawkestf)
generate_lua_lib(fawkestf "${LUA_SOURCES}" "${LUA_LINK_LIBS}")
