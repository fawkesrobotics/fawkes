#*****************************************************************************
#     Makefile Build System for Fawkes: block paths based on laser clusters
#                            -------------------
#   Created on Sun Jul 13 15:19:29 2014
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDCONFDIR)/navgraph/navgraph.mk
include $(BUILDSYSDIR)/eigen3.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_navgraph_clusters = fawkescore fawkesutils fawkesaspects fawkesinterface \
		fawkesblackboard fawkestf \
		fawkesnavgraph fawkesnavgraphaspect \
		Position3DInterface
OBJS_navgraph_clusters = navgraph_clusters_plugin.o navgraph_clusters_thread.o \
			 clusters_block_constraint.o \
			 clusters_static_cost_constraint.o \
			 clusters_distance_cost_constraint.o
OBJS_all    = $(OBJS_navgraph_clusters)

ifeq ($(HAVE_CPP11)$(HAVE_TF)$(HAVE_EIGEN3)$(HAVE_NAVGRAPH),1111)
  CFLAGS  += $(CFLAGS_TF)  $(CFLAGS_EIGEN3)
  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_EIGEN3)

  PLUGINS_all = $(PLUGINDIR)/navgraph-clusters.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS = warning_tf
  endif
  ifneq ($(HAVE_EIGEN3),1)
    WARN_TARGETS = warning_eigen3
  endif
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11 warning_tf warning_eigen3 warning_navgraph
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-clusters plugin$(TNORMAL) (tf not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-clusters plugin$(TNORMAL) (C++11 support required)"
warning_eigen3:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-clusters plugin$(TNORMAL) (Eigen3 not found)"
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph-clusters plugin$(TNORMAL) ($(NAVGRAPH_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
