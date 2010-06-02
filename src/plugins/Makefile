#*****************************************************************************
#                  Makefile Build System for Fawkes: Plugins
#                            -------------------
#   Created on Mon Dec 04 14:54:17 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../..
include $(BASEDIR)/etc/buildsys/config.mk

# base + hardware drivers + functional + integration
SUBDIRS	= bbsync bblogger webview ttmainloop \
	  laser flite joystick katana pantilt \
	  skiller luaagent worldmodel laserht \
	  refboxcomm player xmlrpc \
	  readylogagent
	  #festival

include $(BUILDSYSDIR)/rules.mk

luaagent: skiller

