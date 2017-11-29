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
include $(BUILDCONFDIR)/tf/tf.mk

CFLAGS += $(ECLIPSE_CFLAGS)
LDFLAGS += $(ECLIPSE_LDFLAGS)
include $(BUILDSYSDIR)/boost.mk

REQ_BOOST_LIBS = system filesystem regex
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

LIBS_eclipse_clp = fawkescore fawkesinterface fawkesblackboard \
                     fawkesaspects fawkesutils eclipse TestInterface \
                     EclipseDebuggerInterface ExitSimulationInterface
OBJS_eclipse_clp = plugin.o control_thread.o eclipse_thread.o blackboard_listener_thread.o \
		     externals/fawkes_logger.o       \
		     externals/blackboard.o          \
		     externals/eclipse_path.o        \
		     externals/quaternion_helper.o   \
		     externals/eclipseclp_config.o

ifeq ($(HAVE_NAVGRAPH)$(HAVE_TF),11)
  CFLAGS +=  $(CFLAGS_NAVGRAPH) $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_NAVGRAPH) $(LDFLAGS_TF)
  LIBS_eclipse_clp += fawkesnavgraph
  OBJS_eclipse_clp += externals/rcsoft_map_graph.o
else
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS += warning_navgraph
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif


OBJS_all = $(OBJS_eclipse_clp)

ifeq ($(HAVE_ECLIPSE)$(HAVE_BOOST_LIBS),11)
  PLUGINS_all = $(PLUGINDIR)/eclipse-clp.so

  CFLAGS  += $(ECLIPSE_CFLAGS)  $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(ECLIPSE_LDFLAGS) $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))
else
  ifneq ($(HAVE_ECLIPSE),1)
    WARN_TARGETS += warning_eclipse
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)

.PHONY: warning_eclipse warning_navgraph warning_boost_%
warning_eclipse:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ECLiPSe CLP Plugin$(TNORMAL) ($(ECLIPSE_FAIL_REASON))"
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ECLiPSe navgraph support$(TNORMAL) ($(NAVGRAPH_ERROR))"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build ECLiPSe CLP Plugin $(TNORMAL) (Boost library $* not found)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ECLiPSe navgraph support$(TNORMAL) (TF not available))"

endif

include $(BUILDSYSDIR)/base.mk
