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
include $(BUILDSYSDIR)/eigen3.mk


LIBS_libfawkesnavgraphgenerators = fawkescore fawkesaspects fawkesutils \
				   fawkesnavgraph
OBJS_libfawkesnavgraphgenerators = generator.o voronoi.o grid.o

OBJS_all    = $(OBJS_libfawkesnavgraphgenerators)

ifeq ($(HAVE_NAVGRAPH)$(HAVE_CGAL)$(HAVE_EIGEN3),111)
  CFLAGS  += $(CFLAGS_NAVGRAPH)  $(CFLAGS_CGAL)  $(CFLAGS_EIGEN3)
  LDFLAGS += $(LDFLAGS_NAVGRAPH) $(LDFLAGS_CGAL) $(LDFLAGS_EIGEN3)
  LIBS_all = $(LIBDIR)/libfawkesnavgraphgenerators.$(SOEXT)
else
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
  ifneq ($(HAVE_CGAL),1)
    WARN_TARGETS += warning_cgal
  endif
  ifneq ($(HAVE_EIGEN3),1)
    WARN_TARGETS += warning_eigen3
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_navgraph warning_cgal warning_eigen3
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph generators library cannot be built$(TNORMAL) ($(NAVGRAPH_ERROR))"
warning_cgal:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph generators library cannot be built$(TNORMAL) (CGAL: $(CGAL_ERROR))"
warning_eigen3:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph generators library cannot be built$(TNORMAL) (Eigen3 not found)"
endif


include $(BUILDSYSDIR)/base.mk

