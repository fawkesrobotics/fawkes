#!/bin/sh

set -eu

FILE=$1

NUM_WRONG_LINES=$(clang-format -output-replacements-xml $FILE | grep 'replacement offset' | wc -l)
if [ $NUM_WRONG_LINES != 0 ]; then
	echo $FILE
fi

exit 0
