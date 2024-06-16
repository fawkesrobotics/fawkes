#!/bin/bash

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
#
# Helper script for cmake to run integration test scripts
# args: 1. basedir

if [ -d "$1/tests.d/" ] ; then
	for test in $1/tests.d/* ; do
		if [ -x "$test" ] ; then
			echo -e "[TEST] Executing test $test";
			exec $test | sed s'/^/[TEST] --> /';
			if [ "${PIPESTATUS[0]}"  -ne 0 ] ; then
				echo -e "[TEST] Failed test: $test";
				exit 1;
			fi
		else
			echo -e "[TEST] Skipping $test (not executable)";
		fi
	done
fi
