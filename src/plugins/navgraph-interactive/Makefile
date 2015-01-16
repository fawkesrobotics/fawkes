#*****************************************************************************
#     Makefile Build System for Fawkes: Interactive NavGraph Editing
#                            -------------------
#   Created on Thu Jan 15 16:17:42 2015
#   copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDSYSDIR)/ros.mk

LIBS_navgraph_interactive = fawkescore fawkesutils fawkesaspects fawkesinterface \
			    fawkesblackboard fawkestf fawkesrosaspect \
			    fawkesnavgraphaspect fawkesnavgraph fawkesnavgraphgenerators \
			    NavigatorInterface
OBJS_navgraph_interactive = navgraph_interactive_plugin.o navgraph_interactive_thread.o
OBJS_all    = $(OBJS_navgraph_interactive)

_REQ_ROS_PKGS=visualization_msgs interactive_markers

ifeq ($(HAVE_ROS),1)
  HAVE_ROS_PKGS=$(call ros-have-pkgs,$(_REQ_ROS_PKGS))
endif

ifeq ($(HAVE_NAVGRAPH)$(HAVE_TF)$(HAVE_ROS)$(HAVE_ROS_PKGS),1111)
  CFLAGS  = $(CFLAGS_TF)  $(CFLAGS_NAVGRAPH)  $(CFLAGS_ROS) \
	    $(call ros-pkgs-cflags,$(_REQ_ROS_PKGS))
  LDFLAGS = $(LDFLAGS_TF) $(LDFLAGS_NAVGRAPH) $(LDFLAGS_ROS) \
	    $(call ros-pkgs-lflags,$(_REQ_ROS_PKGS))

  PLUGINS_all = $(PLUGINDIR)/navgraph-interactive.$(SOEXT)
else
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_ROS),1)
    WARN_TARGETS += warning_ros
  endif
  ifneq ($(HAVE_ROS_PKGS),1)
    WARN_TARGETS += warning_ros_pkgs
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_navgraph warning_tf warning_ros warning_ros_pkgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-interactive plugin$(TNORMAL) (tf not available)"
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-interactive plugin$(TNORMAL) (C++11 support required)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-interactive plugin$(TNORMAL) (ROS not available)"
warning_ros_pkgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-interactive plugin$(TNORMAL) (ROS packages missing: $(call ros-missing-pkgs,$(_REQ_ROS_PKGS)))"
endif

include $(BUILDSYSDIR)/base.mk
