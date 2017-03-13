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
include $(BUILDCONFDIR)/navgraph/navgraph.mk

ENABLE_VISUALIZATION = 1


CFLAGS += $(CFLAGS_CPP11)

LIBS_navgraph = m fawkescore fawkesutils fawkesaspects fawkesinterface \
		fawkesblackboard fawkesnavgraph \
		NavigatorInterface NavPathInterface
OBJS_navgraph = navgraph_plugin.o navgraph_thread.o
OBJS_all    = $(OBJS_navgraph)

ifeq ($(HAVE_CPP11)$(HAVE_TF)$(HAVE_NAVGRAPH),111)
  CFLAGS_navgraph_thread  = $(CFLAGS) $(CFLAGS_TF)
  CFLAGS_navgraph_plugin  = $(CFLAGS_navgraph_thread)
  LDFLAGS_navgraph       += $(LDFLAGS_TF)
  LIBS_navgraph          += fawkestf fawkesnavgraphaspect

  PLUGINS_all = $(PLUGINDIR)/navgraph.$(SOEXT)

  SUBDIRS = static-constraints

  ifeq ($(HAVE_ROS),1)

    LIBS_navgraph += fawkesrosaspect
    CFLAGS += $(CFLAGS_ROS) $(CFLAGS_TF)
    LDFLAGS += $(LDFLAGS_ROS) $(LDFLAGS_TF)

    ifeq ($(ENABLE_VISUALIZATION),1)
      ifeq ($(call ros-have-pkg,visualization_msgs),1)
        CFLAGS  += -DHAVE_VISUALIZATION $(call ros-pkg-cflags,visualization_msgs)
        LDFLAGS += $(call ros-pkg-lflags,visualization_msgs)

        OBJS_navgraph += visualization_thread.o
      else
        ENABLE_VISUALIZATION = 0
        WARN_TARGETS += warning_visualization_msgs
      endif
    endif

    ifeq ($(call ros-have-pkg,fawkes_msgs),1)
      CFLAGS  += -DHAVE_FAWKES_MSGS $(call ros-pkg-cflags,fawkes_msgs)
      LDFLAGS += $(call ros-pkg-lflags,fawkes_msgs)

      OBJS_navgraph += rospub_thread.o
    else
      WARN_TARGETS += warning_fawkes_msgs
    endif
  else
    ENABLE_VISUALIZATION = 0
    WARN_TARGETS += warning_ros
  endif
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_tf
  endif
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11 warning_tf warning_ros warning_visualization_msgs warning_navgraph
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (tf not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (C++11 support required)"
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) ($(NAVGRAPH_ERROR))"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling visual debugging$(TNORMAL) (ROS package visualization_msgs not available)"
warning_fawkes_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling non-visual ROS navgraph publishing$(TNORMAL) (ROS package fawkes_msgs not available)"
endif

include $(BUILDSYSDIR)/base.mk
