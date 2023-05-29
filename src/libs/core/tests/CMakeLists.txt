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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(Catch2_FOUND)
  add_executable(test_circular_buffer test_circular_buffer.cpp catch2_main.cpp)
  target_link_libraries(test_circular_buffer stdc++ fawkescore m Catch2::Catch2)
  add_test(NAME CircularBuffer COMMAND test_circular_buffer)
endif()
if(GTest_FOUND)
  add_executable(test_wait_condition test_wait_condition.cpp)
  target_link_libraries(test_wait_condition stdc++ fawkescore pthread
                        GTest::gtest GTest::gtest_main)
  add_test(NAME WaitCondition COMMAND test_wait_condition)
endif()
