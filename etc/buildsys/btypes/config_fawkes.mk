#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings specific to Fawkes
#                            -------------------
#   Created on Tue Apr 10 15:29:29 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

SYSROOT ?=

# Add -DDEBUG_THREADING if you run into threading problems like deadlocks.
# Read FawkesDebugging in the Fawkes Trac Wiki on how to use it
CFLAGS_BASE +=	-g -Wall -Werror

