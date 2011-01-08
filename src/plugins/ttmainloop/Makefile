#*****************************************************************************
#              Makefile Build System for Fawkes: TimeTracker main loop
#                            -------------------
#   Created on Sat Aug 02 13:50:04 2008
#   copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_ttmainloop = fawkescore fawkesutils fawkesaspects
OBJS_ttmainloop = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_ttmainloop)
PLUGINS_all = $(PLUGINDIR)/ttmainloop.so

include $(BUILDSYSDIR)/base.mk

