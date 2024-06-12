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

# Function: depend_on_libcrypto
# Usage: depend_on_libcrypto(TARGET_NAME)
#
# Adds the libcrypto as dependencies to the target.
function(depend_on_libcrypto target)
  optional_depend_on_pkgconfig_libs(${target} libcrypto libcrypto_dep_found)
  if(libcrypto_dep_found)
    target_compile_definitions(${target} PRIVATE HAVE_LIBCRYPTO)
  else()
    optional_depend_on_pkgconfig_libs(${target} openssl openssl_dep_found)
    if(NOT openssl_dep_found)
      build_skipped_message(${target} "encryption support (OpenSSL/libcrypto)")
    endif()
  endif()
endfunction()
