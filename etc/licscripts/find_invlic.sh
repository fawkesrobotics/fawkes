#!/bin/bash

FILE=$1
LIC_SEARCH="license.search_gpl"

for l in $LIC_SEARCH; do
	if perl findlic.pl $l $FILE; then
		echo "++ Find valid license in file $FILE"
		exit 0
	fi
done

echo "** No valid license found in $FILE"
exit 1;

