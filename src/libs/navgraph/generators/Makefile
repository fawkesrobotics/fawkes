#*****************************************************************************
#              Makefile Build System for Fawkes: NavGraph Generators
#                            -------------------
#   Created on Tue Jan 13 10:46:32 2015
#   Copyright (C) 2006-2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDSYSDIR)/cgal.mk


LIBS_libfawkesnavgraphgenerators = fawkescore fawkesaspects fawkesutils \
				   fawkesnavgraph
OBJS_libfawkesnavgraphgenerators = voronoi.o

OBJS_all    = $(OBJS_libfawkesnavgraphgenerators)

ifeq ($(HAVE_NAVGRAPH)$(HAVE_CGAL),11)
  CFLAGS  += $(CFLAGS_NAVGRAPH)  $(CFLAGS_CGAL)
  LDFLAGS += $(LDFLAGS_NAVGRAPH) $(LDFLAGS_CGAL)
  LIBS_all = $(LIBDIR)/libfawkesnavgraphgenerators.$(SOEXT)
else
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
  ifneq ($(HAVE_CGAL),1)
    WARN_TARGETS += warning_cgal
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_navgraph
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph generators library cannot be built$(TNORMAL) ($(NAVGRAPH_ERROR))"
warning_cgal:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph generators library cannot be built$(TNORMAL) (CGAL: $(CGAL_ERROR))"
endif


include $(BUILDSYSDIR)/base.mk

