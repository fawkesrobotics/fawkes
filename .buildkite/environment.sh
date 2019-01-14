#!/usr/bin/env bash

############################################################################
#  Build script environment
#
#  Created: Mon Jan 14 16:38:21 2019 +0100
#  Copyright  2018-2019  Tim Niemueller [www.niemueller.org]
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


# Source standard environment, may setup ccache if available
source /etc/profile

# Determine number of available CPU cores
NPROC=1
if type -p nproc >/dev/null; then
	# Linux
	NPROC=$(nproc)
elif sysctl -n hw.ncpu >/dev/null 2>&1; then
	# FreeBSD
	NPROC=$(sysctl -n hw.ncpu)
fi

# Determine GNU Make executable
MAKE=make
if type -p gmake >/dev/null; then
	MAKE=gmake
fi

