#*****************************************************************************
#                 Makefile Build System for Fawkes: Battery Plugin
#                            -------------------
#   Created on Tue Jan 29 11:59:09 2008
#   copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

LIBS_battery = core utils iowkit
OBJS_battery = battery_plugin.o	\
	       battery_thread.o

OBJS_all    = $(OBJS_battery)
PLUGINS_all = $(PLUGINDIR)/battery.so

include $(BASEDIR)/etc/buildsys/base.mk

