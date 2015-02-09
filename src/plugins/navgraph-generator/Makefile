#*****************************************************************************
#     Makefile Build System for Fawkes: navgraph generation
#                            -------------------
#   Created on Mon Feb 09 17:39:53 2015
#   Copyright (C) 2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/navgraph/navgraph.mk

PRESUBDIRS = interfaces


LIBS_navgraph_generator = fawkescore fawkesutils fawkesaspects	\
			  fawkesinterface fawkesblackboard fawkesnavgraph \
			  fawkesnavgraphaspect fawkesnavgraphgenerators \
			  NavGraphGeneratorInterface
OBJS_navgraph_generator = navgraph_generator_plugin.o navgraph_generator_thread.o
OBJS_all    = $(OBJS_navgraph_generator)

ifeq ($(HAVE_NAVGRAPH)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_CPP11)

  PLUGINS_all = $(PLUGINDIR)/navgraph-generator.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
  # navgraph.mk performs its own warning
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-generator plugin$(TNORMAL) (C++11 support required)"
endif

include $(BUILDSYSDIR)/base.mk
