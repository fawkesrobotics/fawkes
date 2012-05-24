#*****************************************************************************
#     Makefile Build System for Fawkes: BlackBoard Synchronization Plugin
#                            -------------------
#   Created on Wed May 16 16:00:45 2012
#   copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_amcl = fawkescore fawkesutils fawkesaspects fawkesinterface \
	    fawkesblackboard
OBJS_amcl = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

OBJS_all    = $(OBJS_amcl)
PLUGINS_all = $(PLUGINDIR)/amcl.so

include $(BUILDSYSDIR)/base.mk

