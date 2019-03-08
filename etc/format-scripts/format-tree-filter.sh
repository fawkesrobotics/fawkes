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

MERGE_BASE=$1
GIT_COMMIT=$2
PREV=$3

AFFECTED_FILES=$(git diff --diff-filter=AM --name-only $GIT_COMMIT^..$GIT_COMMIT | egrep "\.(h|cpp)$" || :)

if [ -z "$AFFECTED_FILES" ]; then
	exit 0
fi

PREV_AFFECTED_FILES=$(comm -23 <(git diff --name-only $MERGE_BASE..$GIT_COMMIT^ | egrep "\.(h|cpp)$" | sort -u) <(echo $AFFECTED_FILES | sort -u))

echo

NUM_AFFECTED_FILES=0
if [ -n "$AFFECTED_FILES" ]; then
	NUM_AFFECTED_FILES=$(echo $AFFECTED_FILES | tr ' ' '\n' | wc -l)
fi
NUM_PREV_AFFECTED_FILES=0
if [ -n "$PREV_AFFECTED_FILES" ]; then
	NUM_PREV_AFFECTED_FILES=$(echo $PREV_AFFECTED_FILES | tr ' ' '\n' | wc -l)
fi

echo "Checking out: " $NUM_PREV_AFFECTED_FILES
echo "Formatting:   " $NUM_AFFECTED_FILES
#echo "Affected:     " $AFFECTED_FILES

if [ -n "$PREV_AFFECTED_FILES" ]; then
	git checkout $PREV -- $PREV_AFFECTED_FILES
fi

if type -p parallel >/dev/null; then
	parallel -u --will-cite clang-format -i ::: $AFFECTED_FILES
else
	echo "$AFFECTED_FILES" | xargs -P$(nproc) -n1 clang-format -i
fi
