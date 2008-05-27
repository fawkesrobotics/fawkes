#*****************************************************************************
#                 Makefile Build System for Fawkes: Battery Plugin
#                            -------------------
#   Created on Tue Jan 29 11:59:09 2008
#   Copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
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

LIBS_battery = fawkesaspects
OBJS_battery = battery_plugin.o	\
	       battery_thread.o

OBJS_all    = $(OBJS_battery)
PLUGINS_all = $(PLUGINDIR)/battery.so

include $(BASEDIR)/etc/buildsys/base.mk

