#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Tue 07 Jan 2020 16:04:34 CET
#   Copyright (C) 2020 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

LIBS_skiller_simulator_navgraph = fawkescore fawkesutils fawkesaspects \
                                  fawkesblackboard fawkesinterface \
                                  fawkeslogging fawkes_skiller_time_estimator
OBJS_skiller_simulator_navgraph = skiller_simulator_navgraph_plugin.o \
                                  skiller_simulator_navgraph_thread.o \
                                  navgraph_estimator.o

OBJS_all      = $(OBJS_skiller_simulator_navgraph)
PLUGINS_all   = $(PLUGINDIR)/skiller-simulator-navgraph.so

ifeq ($(HAVE_CPP17),1)
  CFLAGS       += $(CFLAGS_CPP17)
  PLUGINS_build = $(PLUGINS_all)
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)

warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting skiller simulator navgraph$(TNORMAL) (C++17 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
