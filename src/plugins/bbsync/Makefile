#*****************************************************************************
#     Makefile Build System for Fawkes: BlackBoard Synchronization Plugin
#                            -------------------
#   Created on Thu Jun 04 15:06:39 2009
#   copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

LIBS_bbsync = fawkescore fawkesutils fawkesaspects fawkesinterface \
	      fawkesblackboard
OBJS_bbsync = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

OBJS_all    = $(OBJS_bbsync)
PLUGINS_all = $(PLUGINDIR)/bbsync.so

include $(BUILDSYSDIR)/base.mk

