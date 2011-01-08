#*****************************************************************************
#   Makefile Build System for Fawkes: Laser Plugin Dead Spots Calibration Tool
#                            -------------------
#   Created on Tue Jun 23 23:28:25 2009
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_fflaser_deadspots = stdc++ m fawkescore fawkesutils fawkesnetcomm \
			 fawkesblackboard fawkesinterface fawkesconfig \
			 Laser360Interface Laser720Interface
OBJS_fflaser_deadspots = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

BINS_all = $(BINDIR)/fflaser_deadspots
OBJS_all = $(OBJS_fflaser_deadspots)
MANPAGES_all = $(MANDIR)/man1/fflaser_deadspots.1

include $(BUILDSYSDIR)/base.mk
