#*****************************************************************************
#                 Makefile Build System for Fawkes: Example Plugin
#                            -------------------
#   Created on Wed Nov 22 17:05:17 2006
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

# to enable time tracking add:
# CFLAGS += -DFVBASE_TIMETRACKER -DFVBASE_TT_PRINT_INT=100
# Set the print interval to the number of loops

LIBS_fvbase = fawkescore fawkesutils fawkesaspects fvutils fvcams \
	      fawkesblackboard fawkesinterface SwitchInterface
OBJS_fvbase = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_fvbase)
PLUGINS_all = $(PLUGINDIR)/fvbase.so

include $(BUILDSYSDIR)/base.mk

