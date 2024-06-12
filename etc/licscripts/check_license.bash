#! /bin/bash
#
# check_license.bash
#
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

set -e
set -u
GOOD_LICENSES=("GPL-2+" "GPL-3" "BSD-3-clause" "BSD-3-clause and/or GPL-2+" "Apache-2.0")
is_good_license () {
  local good_license
  for good_license in "${GOOD_LICENSES[@]}" ; do
    if [[ "$1" == "$good_license" ]] ; then
      return 0
    fi
  done
  return 1;
}

check_files () {
  type -p licensecheck >/dev/null || \
    (echo "Could not find licensecheck binary, please install it!"; exit 1)
  IFS=$'\n'
  local have_bad_license=0
  for line in $(licensecheck -m --deb-fmt $@) ; do
    local file=$(echo "$line" | cut -s -f 1)
    local license=$(echo "$line" | cut -s -f 2)
    if ! is_good_license "$license" ; then
      echo -e "\e[31mBad license header ($license): $file\e[0m"
      have_bad_license=1
    fi
  done
  if [[ $have_bad_license -eq 1 ]] ; then
    echo -e "\e[31m--> Some files do not have a proper license header!\e[0m"
    return 1
  else
    echo -e "\e[32m--> All files have a proper license header!\e[0m"
    return 0
  fi
}
if [ $# -eq 0 ]
  then
    echo "No arguments supplied, running on source tree"
    INPUT=$(git ls-files --full-name {\*.{h,cpp,py},\*CMakeLists.txt} | awk "{ print \"$(git rev-parse --show-toplevel)/\"\$1 }" | git check-attr --stdin ignore-license-check | awk '$3 != "set" { print $1; }' | tr -d ':')
    check_files "$INPUT"
  else
    check_files $@
fi
