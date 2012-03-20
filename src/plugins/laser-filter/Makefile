#*****************************************************************************
#        Makefile Build System for Fawkes: Laser Filter Plugin
#                            -------------------
#   Created on Sun Mar 13 01:05:36 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

SUBDIRS = deadspots

LIBS_laser_filter = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	             fawkesinterface Laser360Interface Laser720Interface fawkesgeometry

OBJS_laser_filter = $(filter-out deadspots/% $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
PLUGINS_all = $(PLUGINDIR)/laser-filter.so

OBJS_all = $(OBJS_laser_filter)

include $(BUILDSYSDIR)/base.mk

