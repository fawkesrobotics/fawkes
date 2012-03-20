#*****************************************************************************
#   Makefile Build System for Fawkes: Config Settings specific to MacOSX
#                            -------------------
#   Created on Sun Feb 06 00:16:23 2011
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

DEFAULT_INCLUDES += -I/opt/local/include

LIBDIRS_BASE    += /opt/local/lib
LDFLAGS_MINIMUM  = $(addprefix -L,$(wildcard $(LIBDIRS_BASE))) -rdynamic -fPIC $(LDFLAGS_OPENMP) -lstdc++
LDFLAGS_SHARED   = -dynamiclib
SOEXT            = dylib

