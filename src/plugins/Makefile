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
SUBDIRS	= bbsync bblogger webview ttmainloop rrd \
	  laser imu flite festival joystick openrave \
	  katana jaco pantilt roomba nao robotino \
	  bumblebee2 realsense perception amcl \
	  skiller luaagent \
	  laser-filter laser-lines laser-cluster laser-pointclouds \
	  static_transforms navgraph navgraph-clusters navgraph-generator colli \
	  clips clips-agent clips-protobuf clips-navgraph \
	  clips-pddl-parser clips-ros clips-tf \
		openprs openprs-agent eclipse-clp \
	  mongodb mongodb_log robot-memory clips-robot-memory pddl-robot-memory \
	  openrave-robot-memory openni refboxcomm ros player xmlrpc gossip \
	  robot_state_publisher gazebo dynamixel navgraph-interactive \
	  pddl-planner stn-generator clips-executive

include $(BUILDSYSDIR)/rules.mk

luaagent: skiller
rrdweb: rrd
katana jaco: openrave
amcl colli laser-lines navgraph navgraph-generator navgraph-interactive perception robotino: ros
mongodb_log: mongodb
mongodb: rrd
clips-navgraph clips-agent clips-pddl-parser clips-protobuf clips-tf clips-robot-memory: clips
clips-navgraph navgraph-clusters: navgraph
clips-ros: clips ros
robot-memory: mongodb
clips-robot-memory: robot-memory
pddl-robot-memory: robot-memory
openrave-robot-memory: robot-memory openrave
pddl-planner: robot-memory
gazebo: robotino
skiller: navgraph
perception: mongodb
navgraph-generator: navgraph amcl
openprs-agent: openprs
laser-filter: amcl
stn-generator: pddl-planner

