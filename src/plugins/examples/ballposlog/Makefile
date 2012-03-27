#*****************************************************************************
#              Makefile Build System for Fawkes: BallPosLog Plugin
#                            -------------------
#   Created on Thu Jan 24 17:11:34 2008
#   copyright (C) 2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_ballposlog = fawkescore fawkesaspects fawkesblackboard fawkesinterface ObjectPositionInterface
OBJS_ballposlog = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_ballposlog)
PLUGINS_all = $(PLUGINDIR)/ballposlog.so

include $(BUILDSYSDIR)/base.mk

