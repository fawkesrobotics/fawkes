#!/usr/bin/env bash

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

set -eu

FILE=$1

if [ ! -e $FILE ] ; then
	exit 0
fi

if ! cmp -s <(clang-format $FILE) $FILE; then
	echo $FILE
fi

exit 0
