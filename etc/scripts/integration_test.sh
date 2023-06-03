#!/bin/bash
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
