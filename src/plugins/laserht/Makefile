#*****************************************************************************
#            Makefile Build System for Fawkes: Laser Plugin
#                            -------------------
#   Created on Sat Jul 04 21:37:27 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id: Makefile 2627 2009-06-25 18:08:09Z tim $
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

LIBS_laserline = fawkescore fawkesutils fawkesaspects \
	     	Laser360Interface Laser720Interface ObjectPositionInterface
OBJS_laserline = $(filter-out deadspots/%,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

PLUGINS_all = $(PLUGINDIR)/laserline.so

OBJS_all = $(OBJS_laserline)

include $(BASEDIR)/etc/buildsys/base.mk
