#*****************************************************************************
#              Makefile Build System for Fawkes: ECLiPSe CLP Plugin
#                            -------------------
#   Created on Wed Jul 15 11:40:13 2009
#   copyright (C) 2009 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/plugins/eclipse-clp/eclipse.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk

CFLAGS += $(ECLIPSE_CFLAGS)
LDFLAGS += $(ECLIPSE_LDFLAGS)

LIBS_eclipse_clp = fawkescore fawkesinterface fawkesblackboard \
                     fawkesaspects fawkesutils eclipse TestInterface \
                     EclipseDebuggerInterface
OBJS_eclipse_clp = plugin.o control_thread.o eclipse_thread.o \
		     externals/fawkes_logger.o       \
		     externals/blackboard.o          \
		     externals/eclipse_path.o

ifeq ($(HAVE_NAVGRAPH),1)
  CFLAGS +=  $(CFLAGS_NAVGRAPH)
  LDFLAGS += $(LDFLAGS_NAVGRAPH)
  LIBS_eclipse_clp +=fawkesnavgraph
  OBJS_eclipse_clp += externals/rcsoft_map_graph.o
else
  WARN_TARGETS += warning_navgraph
endif

#LIBS_eclipse_path = fawkescore eclipse
#OBJS_eclipse_path = externals/eclipse_path.o

OBJS_all = $(OBJS_eclipse_clp)

ifeq ($(HAVE_ECLIPSE),1)
  PLUGINS_all = $(PLUGINDIR)/eclipse-clp.so
  # debug
  #BINS_all = $(BINDIR)/eclipse_path
else
  WARN_TARGETS += warning_eclipse
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_eclipse
warning_eclipse:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ECLiPSe CLP Plugin$(TNORMAL) ($(ECLIPSE_FAIL_REASON))"
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ECLiPSe navgraph support$(TNORMAL) ($(NAVGRAPH_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
