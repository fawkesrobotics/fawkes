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
	  laser imu flite festival joystick openrave \
	  katana pantilt roomba nao robotino \
	  bumblebee2 perception amcl \
	  skiller luaagent worldmodel \
	  laser-filter laser-lines laser-cluster laser-pointclouds \
	  static_transforms navgraph navgraph-clusters navgraph-generator colli \
	  clips clips-agent clips-protobuf clips-webview clips-navgraph clips-ros \
	  clips-tf openprs openprs-agent \
	  mongodb mongodb_log \
	  openni refboxcomm ros player xmlrpc gossip \
	  robot_state_publisher gazebo dynamixel
	  # readylogagent

include $(BUILDSYSDIR)/rules.mk

luaagent: skiller
rrdweb: rrd
katana: openrave
amcl colli navgraph perception robotino: ros
mongodb_log: mongodb
mongodb: rrd
clips-navgraph clips-agent clips-protobuf clips-tf: clips
clips-navgraph navgraph-clusters: navgraph
clips-webview: clips webview
clips-ros: clips ros
gazebo: robotino
skiller: navgraph
perception: mongodb
navgraph-generator: navgraph amcl
openprs-agent: openprs

