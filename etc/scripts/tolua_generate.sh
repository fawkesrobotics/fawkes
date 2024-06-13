#!/bin/bash

# Copyright (C) 2021 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.

# Helper script for cmake to generate c++ lua bindings
# args: 1. output path
# args: 2. target
# args: 3. toluaext path
# args: 4. GPL_WRE license
# args: 5. path to tolua files
# args: 6.-n. names of tolua files.

output_path=$1
shift
target=$1
shift
toluaext=$1
shift
license=$1
shift
tolua_file_path=$1
shift
cat "${tolua_file_path}/$1" > "${output_path}/${target}_tolua.pkg"
shift
while (( "$#" )); do
  cat "${tolua_file_path}/$1" >> "${output_path}/${target}_tolua.pkg"
  shift
done
tolua++ -L "${toluaext}" -n "${target}" "${output_path}/${target}_tolua.pkg" | \
sed -e 's/^\(.*Generated automatically .*\) on .*$/\1/' | \
awk '/^#if defined/ { f=1 }; f { t = t "\n" $0 }; !f {print}; f && /^#endif/ {print "extern \"C\" {" t "\n}\n"; f=0}' | \
awk "/^\*\/$/ { print; while ((getline line < \"${license}\") > 0) print line; print \"\n#include <core/exception.h>\" }; ! /^\*\/$/ { print }" \
> "${output_path}/${target}_tolua.cpp"
