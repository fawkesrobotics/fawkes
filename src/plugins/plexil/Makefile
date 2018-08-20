#*****************************************************************************
#           Makefile Build System for Fawkes: PLEXIL Executive Plugin
#                            -------------------
#   Created on Mon Aug 13 11:21:49 2018
#   Copyright (C) 2006-2018 by Tim Niemueller
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
include $(BUILDSYSDIR)/clips.mk
include $(BUILDSYSDIR)/boost.mk
include $(LIBSRCDIR)/navgraph/navgraph.mk

PLEXIL_REQ_BOOST_LIBS = system filesystem
HAVE_PLEXIL_BOOST_LIBS = $(call boost-have-libs,$(OPENPRS_REQ_BOOST_LIBS))

LIBS_plexil = \
	fawkescore fawkesutils fawkesaspects fawkesblackboard \
	fawkesinterface fawkes_protobuf_comm \
  PlexilAppFramework PlexilPlanDebugListener PlexilLuvListener PlexilSockets \
  SkillerInterface
OBJS_plexil = plexil_plugin.o plexil_thread.o clock_adapter.o timer_thread.o \
  log_adapter.o be_adapter.o thread_adapter.o protobuf_adapter.o

OBJS_all = $(OBJS_plexil)



ifeq ($(HAVE_PLEXIL)$(HAVE_PLEXIL_BOOST_LIBS),11)
  CFLAGS  += $(CFLAGS_PLEXIL)  $(call boost-libs-cflags,$(PLEXIL_REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PLEXIL) $(call boost-libs-ldflags,$(PLEXIL_REQ_BOOST_LIBS))

  ifeq ($(HAVE_NAVGRAPH),1)
    LIBS_plexil += fawkesnavgraphaspect Position3DInterface
    OBJS_plexil += navgraph_access_thread.o navgraph_adapter.o
    CFLAGS += -DHAVE_NAVGRAPH
  else
    WARN_TARGETS += warning_navgraph
  endif

  PLUGINS_all = $(PLUGINDIR)/plexil.so
else
  ifneq ($(HAVE_PLEXIL),1)
    WARN_TARGETS += warning_plexil
  endif
  #ifneq ($(HAVE_CPP17),1)
  #  WARN_TARGETS += warning_cpp17
  #endif
  ifneq ($(HAVE_OPENPRS_BOOST_LIBS),1)
    PLEXIL_ERROR += Boost libraries missing
    PLEXIL_WARN_TARGETS_BOOST = $(foreach l,$(PLEXIL_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_plexil_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_plexil warning_cpp17 $(PLEXIL_WARN_TARGETS_BOOST) warning_navgraph
warning_plexil:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting plexil plugin$(TNORMAL) ($(PLEXIL_ERROR))"
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting plexil plugin$(TNORMAL) (C++17 required)"
  ifneq ($(PLEXIL_WARN_TARGETS_BOOST),)
$(PLEXIL_WARN_TARGETS_BOOST): warning_plexil_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting plexil plugin$(TNORMAL) (Boost library $* not found)"
  endif
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting navgraph support for plexil$(TNORMAL) (navgraphaspect not available)"
endif

include $(BUILDSYSDIR)/base.mk
