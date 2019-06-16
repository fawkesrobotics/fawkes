#!/bin/bash

#  Copyright (c) 2018 Tim Niemueller [www.niemueller.de]
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

ALL_FILES=$(git ls-files \*.{h,cpp})
if type -p parallel >/dev/null; then
	OFFENDING_FILES=$(parallel --will-cite --bar $SCRIPT_PATH/check-file.sh ::: $ALL_FILES)
else
	OFFENDING_FILES=$(echo "$ALL_FILES" | xargs -P$(nproc) -n1 $SCRIPT_PATH/check-file.sh)
fi

if [ "$OFFENDING_FILES" != "" ]; then
	echo $OFFENDING_FILES
	exit 1
else
	exit 0
fi
