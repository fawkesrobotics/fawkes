#*****************************************************************************
#     Makefile Build System for Fawkes: configurable static constraints
#                            -------------------
#   Created on Fri Jul 11 17:26:49 2014
#   copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/ros.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_navgraph_static_constraints = fawkescore fawkesutils fawkesnavgraph fawkesaspects \
				fawkesnavgraphaspect
OBJS_navgraph_static_constraints = navgraph_stconstr_plugin.o navgraph_stconstr_thread.o
OBJS_all    = $(OBJS_navgraph_static_constraints)

ifeq ($(HAVE_CPP11),1)
  LIBS_navgraph      += fawkestf fawkesnavgraphaspect

  PLUGINS_all = $(PLUGINDIR)/navgraph-static-constraints.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph plugin$(TNORMAL) (C++11 support required)"
endif

include $(BUILDSYSDIR)/base.mk
