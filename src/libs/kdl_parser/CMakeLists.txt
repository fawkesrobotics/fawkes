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
  fawkeskdl_parser SHARED
  # tests/test_kdlparser.cpp
  model.cpp kdl_parser.cpp)
target_link_libraries(fawkeskdl_parser fawkescore)
depend_on_kdl_parser(fawkeskdl_parser)
depend_on_eigen3(fawkeskdl_parser)
target_compile_features(fawkeskdl_parser PUBLIC cxx_std_11)
