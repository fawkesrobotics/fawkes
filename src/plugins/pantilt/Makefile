
#*****************************************************************************
#            Makefile Build System for Fawkes: PanTilt Plugin
#                            -------------------
#   Created on Wed Jun 17 22:18:13 2009
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
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

LIBS_pantilt = fawkescore fawkesutils fawkesaspects PanTiltInterface LedInterface
OBJS_pantilt = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))

PLUGINS_all = $(PLUGINDIR)/pantilt.so

OBJS_all = $(OBJS_pantilt)

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BASEDIR)/etc/buildsys/interface.mk

include $(BASEDIR)/etc/buildsys/base.mk

