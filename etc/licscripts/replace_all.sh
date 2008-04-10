#!/bin/bash

DIRS="$@"

for d in $DIRS; do
	echo "Processing $d"
	for ext in h cpp; do
		echo "Replacing in .$ext files"
		find $d -name "*.$ext" -exec ./replace_license.pl {} \;
	done
done

