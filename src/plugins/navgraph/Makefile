#*****************************************************************************
#     Makefile Build System for Fawkes: Graph-based global path planning
#                            -------------------
#   Created on Tue Sep 18 15:52:22 2012
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
include $(BUILDSYSDIR)/ros.mk

ENABLE_VISUALIZATION = 1

CFLAGS += -std=c++0x

LIBS_navgraph = fawkescore fawkesutils fawkesaspects fawkesinterface \
	    fawkesblackboard NavigatorInterface
OBJS_navgraph = navgraph_plugin.o navgraph_thread.o search_state.o

OBJS_all    = $(OBJS_navgraph)

ifeq ($(HAVE_TF),1)
  CFLAGS_navgraph_thread  = $(CFLAGS) $(CFLAGS_TF)
  CFLAGS_navgraph_plugin  = $(CFLAGS_navgraph_thread)
  LDFLAGS_navgraph       += $(LDFLAGS_TF)
  LIBS_navgraph          += fawkestf

  PLUGINS_all = $(PLUGINDIR)/navgraph.$(SOEXT)


  ifeq ($(ENABLE_VISUALIZATION),1)
    ifeq ($(HAVE_ROS),1)
      ifeq ($(call ros-have-pkg,visualization_msgs),1)
        CFLAGS  += -DHAVE_VISUALIZATION $(CFLAGS_ROS)
	CFLAGS_visualization_thread = $(CFLAGS) $(CFLAGS_TF) \
				      $(call ros-pkg-cflags,visualization_msgs)
        LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs)

        LIBS_navgraph += fawkesrosaspect

        OBJS_navgraph += visualization_thread.o
    else
      ENABLE_VISUALIZATION = 0
      WARN_TARGETS += warning_visualization_msgs
    endif
  else
    ENABLE_VISUALIZATION = 0
    WARN_TARGETS += warning_ros
  endif
endif

else
  WARN_TARGETS = warning_tf
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_ros warning_visualization_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (tf not available)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS package visualization_msgs not available)"
endif

include $(BUILDSYSDIR)/base.mk
