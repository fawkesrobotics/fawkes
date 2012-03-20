
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
include $(BASEDIR)/etc/buildsys/config.mk

SUBDIRS=console

LIBS_bblogger = fawkescore fawkesutils fawkesaspects fawkesinterface \
		fawkesblackboard SwitchInterface
OBJS_bblogger = bblogger_plugin.o log_thread.o


LIBS_bblogreplay = fawkescore fawkesutils fawkesaspects fawkesinterface \
		   fawkesblackboard
OBJS_bblogreplay = bblogreplay_plugin.o		\
		   logreplay_thread.o		\
		   logreplay_bt_thread.o	\
		   bblogfile.o


OBJS_all    = $(OBJS_bblogger) $(OBJS_bblogreplay)
PLUGINS_all = $(PLUGINDIR)/bblogger.so \
	      $(PLUGINDIR)/bblogreplay.so

include $(BUILDSYSDIR)/base.mk

