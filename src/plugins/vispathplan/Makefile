#*****************************************************************************
#       Makefile Build System for Fawkes: Pathplan Visualization Plugin
#                            -------------------
#   Created on Fri Nov 11 21:07:27 2011
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

LIBS_vispathplan = m fawkescore fawkesutils fawkesaspects fawkesrosaspect
OBJS_vispathplan = vispathplan_plugin.o vispathplan_thread.o

OBJS_all    = $(OBJS_vispathplan)

ifeq ($(HAVE_ROS),1)
  ifeq ($(call ros-have-pkg,visualization_msgs),1)
    PLUGINS_all = $(PLUGINDIR)/vispathplan.so

    CFLAGS  += $(CFLAGS_ROS) $(call ros-pkg-cflags,visualization_msgs)
    LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)
  else
    WARN_TARGETS += warning_visualization_msgs
  endif
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_visualization_msgs
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling pathplan visualization plugin$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling pathplan visualization plugin$(TNORMAL) (ROS package visualization_msgs not available)"
endif


include $(BUILDSYSDIR)/base.mk

