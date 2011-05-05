#*****************************************************************************
#               Makefile Build System for Fawkes: ROS Plugin
#                            -------------------
#   Created on Thu May 05 18:23:31 2011
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
include $(BUILDSYSDIR)/ros.mk

LIBS_ros = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect
OBJS_ros = ros_plugin.o node_thread.o

LIBS_ros_talkerpub = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect
OBJS_ros_talkerpub = talkerpub_plugin.o talkerpub_thread.o

OBJS_all = $(OBJS_ros) $(OBJS_ros_talkerpub)

ifeq ($(HAVE_ROS),1)
  PRESUBDIRS += aspect

  CFLAGS  += $(CFLAGS_ROS)
  LDFLAGS += $(LDFLAGS_ROS)

  CFLAGS_talkerpub_thread += $(shell rospack export --lang=cpp --attrib=cflags std_msgs)
  LDFLAGS_ros_talkerpub += $(LDFLAGS) $(shell rospack export --lang=cpp --attrib=lflags std_msgs)

  PLUGINS_all = $(PLUGINDIR)/ros.so $(PLUGINDIR)/ros-talkerpub.so
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (ROS not found)"
endif

include $(BUILDSYSDIR)/base.mk

