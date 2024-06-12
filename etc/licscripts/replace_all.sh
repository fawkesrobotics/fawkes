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

DIRS="$@"

for d in $DIRS; do
	echo "Processing $d"
	for ext in h cpp; do
		echo "Replacing in .$ext files"
		find $d -name "*.$ext" -exec ./replace_license.pl {} \;
	done
done
