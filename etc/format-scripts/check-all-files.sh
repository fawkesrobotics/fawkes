#!/bin/bash

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

ALL_FILES=$(git ls-files *.{h,cpp})
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
