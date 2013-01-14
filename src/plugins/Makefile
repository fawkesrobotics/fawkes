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

# base + hardware drivers + perception + functional + integration
SUBDIRS	= bbsync bblogger webview ttmainloop rrd rrdweb \
	  laser flite festival joystick openrave \
	  katana pantilt roomba nao robotino \
	  perception amcl \
	  skiller luaagent worldmodel laserht laser-filter laser-pointclouds \
	  static_transforms navgraph clips clips-agent \
	  openni refboxcomm ros player xmlrpc \
	  readylogagent

include $(BUILDSYSDIR)/rules.mk

luaagent: skiller
rrdweb: rrd
katana: openrave
perception: ros
amcl: ros
clips-agent: clips

