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

function(depend_on_cgal_libs target)
  if(NOT CGAL IN_LIST FAWKES_DEPENDENCIES_CHECKED)
    find_package(CGAL)
    remember_dependency(CGAL)
  endif()

  depend_on_pkgconfig_libs(${target} "gmp;mpfr")
  depend_on_boost_libs(${target} "system;thread")
  target_include_directories(${target} PUBLIC ${Boost_INCLUDE_DIR})
  target_link_libraries(${target} ${CGAL_LDLAGS} ${GMP_LDFLAGS} ${MPFR_LDFLAGS}
                        -lm)
  target_compile_options(
    ${target}
    PUBLIC ${CGAL_LDLAGS}
           ${GMP_LDFLAGS}
           ${MPFR_LDFLAGS}
           -DHAVE_CGAL
           -Wno-depcrecated-register
           -DCGAL_DISABLE_ROUNDING_MATH_CHECK
           -frounding-math)
  # TODO: clang flags: -Wno-unused-local-typedef
endfunction()
