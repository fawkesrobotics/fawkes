
#*****************************************************************************
#            Makefile Build System for Fawkes: PanTilt Plugin
#                            -------------------
#   Created on Thu Dec 30 22:01:16 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_roomba = fawkescore fawkesutils fawkesaspects
OBJS_roomba = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))

PLUGINS_all = $(PLUGINDIR)/roomba.so

OBJS_all = $(OBJS_roomba)

#INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
#include $(BUILDSYSDIR)/interface.mk

include $(BUILDSYSDIR)/base.mk

