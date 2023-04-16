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

set(urg_gbx_deps "flexiport;hokuyoaist")
function(depend_on_laser target)
  optional_depend_on_pkgconfig_libs(${target} "${urg_gbx_deps}"
                                    urg_gbx_deps_found)
  optional_depend_on_pkgconfig_libs(${target} libudev libudev_dep_found)
  optional_depend_on_pkgconfig_libs(${target} libusb-1.0 libusb-1.0_dep_found)
  if(urg_gbx_deps_found)
    target_compile_definitions(${target} PUBLIC HAVE_URG_GBX)
    set(URG_GBX_FOUND
        1
        PARENT_SCOPE)
  endif()
  if(libudev_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_LIBUDEV)
    set(LIBUDEV_FOUND
        1
        PARENT_SCOPE)
  endif()
  if(libusb-1.0_dep_found)
    target_compile_definitions(${target} PUBLIC HAVE_LIBUSB)
    set(LIBUSB_FOUND
        1
        PARENT_SCOPE)
  endif()

  if(URG_ROOT)
    target_link_libraries(${target} liburg)
  endif()
  if(SICK_TIM55X_BOOST_DEPS_FOUND)
    target_compile_definitions(${target} PUBLIC HAVE_SICK55X_BOOST)
    target_link_libraries(${target} sick_tim55x)
  endif()
endfunction()
