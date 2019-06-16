#!/bin/bash

#  Copyright (c) 2018-2019 Tim Niemueller [www.niemueller.de]
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

DIFF=diff
if type -p gdiff >/dev/null; then
  DIFF=gdiff
fi

if [ "${SHOW_PROGRESS:-}" == "no" ]; then
	SHOW_PROGRESS=
else
	SHOW_PROGRESS="yes";
fi

MERGE_BASE=$($DIFF --old-line-format='' --new-line-format='' \
             <(git rev-list --first-parent "HEAD") \
             <(git rev-list --first-parent "origin/master") \
             | head -1)

AFFECTED_FILES=$(git diff --name-only $MERGE_BASE HEAD | grep -E '\.(cpp|h)$' | paste -sd " " -)
MODIFIED_FILES=$(git ls-files -m \*.{h,cpp})

# If no (C++) files are affected we are done
if [ -z "$AFFECTED_FILES" -a -z "$MODIFIED_FILES" ]; then
	exit 0
fi

FILES=$(echo "$AFFECTED_FILES" "$MODIFIED_FILES" | tr ' ' '\n' | sort -u)

if type -p parallel >/dev/null; then
	OFFENDING_FILES=$(parallel --will-cite ${SHOW_PROGRESS:+--bar} $SCRIPT_PATH/check-file.sh ::: $FILES)
else
	OFFENDING_FILES=$(echo "$FILES" | xargs -P$(nproc) -n1 $SCRIPT_PATH/check-file.sh)
fi

if [ "$OFFENDING_FILES" != "" ]; then
	echo $FILES | wc -w
	echo $OFFENDING_FILES
	exit 1
else
	exit 0
fi
