#!/bin/bash

############################################################################
#  Conditional webview frontend build step
#
#  Created: Sun Nov 11 21:49:35 2018 +0100
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

# Build webview, if master or webview branch, or webview files modified
BUILD_WEBVIEW=
if [[ ${BUILDKITE_BRANCH:-master} =~ ^(master|.*webview.*)$ ]]; then
	BUILD_WEBVIEW=1
fi

if [[ ":$AFFECTED_FILES:" == *":src/plugins/webview/frontend/"*:* ]]; then
	BUILD_WEBVIEW=1
fi

if [ -n "$BUILD_WEBVIEW" ]; then
	cat $SCRIPT_PATH/webview-frontend.yml
fi
