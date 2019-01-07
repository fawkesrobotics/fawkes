#!/usr/bin/env bash

############################################################################
#  Conditional FreeBSD build step
#
#  Created: Mon Jan 07 18:54:45 2019 +0100
#  Copyright  2018  Tim Niemueller [www.niemueller.org]
############################################################################

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

# Error out on an error in the script
set -euo pipefail

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# Allow skipping FreeBSD altogether, e.g., in domain repos
if [ -n "${SKIP_FREEBSD:-}" ]; then
	exit 0
fi

# Build FreeBSD, if master or freebsd branch, or for pull request
BUILD_FREEBSD=
if [[ ${BUILDKITE_BRANCH:-master} =~ ^(master|.*freebsd.*)$ ]]; then
	BUILD_FREEBSD=1
fi

if [[ "$BUILDKITE_PULL_REQUEST" != "false" ]]; then
	BUILD_FREEBSD=1
fi

if [ -n "$BUILD_FREEBSD" ]; then
	cat $SCRIPT_PATH/10-freebsd.yml
fi
