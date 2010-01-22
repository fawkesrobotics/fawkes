#*****************************************************************************
#         Makefile Build System for Fawkes: BlackBoard Logger Plugin
#                            -------------------
#   Created on Sun Nov 08 00:39:22 2009
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

SUBDIRS=console

LIBS_bblogger = fawkesaspects fawkesinterface fawkesgeometry fawkesblackboard
OBJS_bblogger = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

OBJS_all    = $(OBJS_bblogger)
PLUGINS_all = $(PLUGINDIR)/bblogger.so

include $(BASEDIR)/etc/buildsys/base.mk

