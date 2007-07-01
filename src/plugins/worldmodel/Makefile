#*****************************************************************************
#              Makefile Build System for Fawkes: WorldModel Plugin
#                            -------------------
#   Created on Fir Jun 29 11:53:13 2007 (on flight to RoboCup 2007, Atlanta)
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../..

LIBS_worldmodel = core utils netcomm
OBJS_worldmodel = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_worldmodel)
PLUGINS_all = $(PLUGINDIR)/worldmodel.so

include $(BASEDIR)/etc/buildsys/base.mk

