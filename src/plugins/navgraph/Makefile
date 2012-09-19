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
else
  WARN_TARGETS = warning_tf
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk
