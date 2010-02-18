#*****************************************************************************
#         Makefile Build System for Fawkes: BlackBoard Logger Replay Plugin
#                            -------------------
#   Created on Thu Feb 10 15:57:22 2009
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


LIBS_bblogger_replay = fawkesaspects fawkesinterface fawkesgeometry fawkesblackboard
OBJS_bblogger_replay = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

OBJS_all    = $(OBJS_bblogger_replay)
PLUGINS_all = $(PLUGINDIR)/bblogger_replay.so


include $(BASEDIR)/etc/buildsys/base.mk

