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
include $(BUILDSYSDIR)/protobuf.mk
include $(LIBSRCDIR)/navgraph/navgraph.mk

PLEXIL_REQ_BOOST_LIBS = system filesystem interprocess/sync/file_lock
HAVE_PLEXIL_BOOST_LIBS = $(call boost-have-libs,$(OPENPRS_REQ_BOOST_LIBS))

LIBS_plexil = \
	fawkescore fawkesutils fawkesaspects fawkesblackboard \
  PlexilAppFramework PlexilPlanDebugListener PlexilLuvListener PlexilSockets
OBJS_plexil = plexil_plugin.o plexil_thread.o

OBJS_plexil_FawkesTimeAdapter = clock_adapter.o timer_thread.o
LIBS_plexil_FawkesTimeAdapter = fawkescore fawkesutils PlexilAppFramework PlexilIntfc PlexilUtils
NOSOVER_plexil_FawkesTimeAdapter = 1

OBJS_plexil_FawkesLoggingAdapter = log_adapter.o
LIBS_plexil_FawkesLoggingAdapter = fawkescore fawkesutils PlexilAppFramework PlexilValue PlexilIntfc PlexilExec
NOSOVER_plexil_FawkesLoggingAdapter = 1

OBJS_plexil_BehaviorEngineAdapter = be_adapter.o
LIBS_plexil_BehaviorEngineAdapter = fawkescore fawkesutils fawkesblackboard fawkesinterface SkillerInterface \
                                    PlexilAppFramework PlexilValue PlexilIntfc PlexilExec
NOSOVER_plexil_BehaviorEngineAdapter = 1

OBJS_plexil_ThreadName = thread_adapter.o
LIBS_plexil_ThreadName = fawkescore \
                         PlexilAppFramework PlexilIntfc pugixml
NOSOVER_plexil_ThreadName = 1

OBJS_plexil_ProtobufCommAdapter = protobuf_adapter.o
LIBS_plexil_ProtobufCommAdapter = fawkescore fawkesutils \
                                  PlexilAppFramework PlexilValue PlexilIntfc PlexilUtils pugixml
CFLAGS_protobuf_adapter            = $(CFLAGS) $(CFLAGS_PROTOBUF) $(CFLAGS_PROTOBUF_COMM)
LDFLAGS_plexil_ProtobufCommAdapter = $(LDFLAGS) $(LDFLAGS_PROTOBUF) $(LDFLAGS_PROTOBUF_COMM)
NOSOVER_plexil_ProtobufCommAdapter = 1

OBJS_plexil_NavGraphAdapter = navgraph_adapter.o
LIBS_plexil_NavGraphAdapter = fawkescore fawkesutils fawkesblackboard \
                              fawkesinterface Position3DInterface fawkesnavgraph \
                              PlexilAppFramework PlexilValue PlexilIntfc PlexilUtils
NOSOVER_plexil_NavGraphAdapter = 1

OBJS_plexil_FawkesRemoteAdapter = remote_adapter.o
LIBS_plexil_FawkesRemoteAdapter = fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
                              fawkesconfig fawkeslogging fawkesnavgraph pthread \
                              PlexilAppFramework PlexilValue PlexilIntfc PlexilUtils pugixml
NOSOVER_plexil_FawkesRemoteAdapter = 1

OBJS_plexil_FawkesConfigurationAdapter = config_adapter.o
LIBS_plexil_FawkesConfigurationAdapter = fawkescore fawkesutils \
                                         fawkesconfig fawkeslogging \
                                         PlexilAppFramework PlexilValue PlexilIntfc PlexilUtils
NOSOVER_plexil_FawkesConfigurationAdapter = 1

OBJS_plexil_FawkesBlackboardAdapter = blackboard_adapter.o
LIBS_plexil_FawkesBlackboardAdapter = fawkescore fawkesutils fawkesblackboard fawkesinterface \
                                      fawkesconfig fawkeslogging \
                                      PlexilAppFramework PlexilValue PlexilIntfc PlexilUtils
NOSOVER_plexil_FawkesBlackboardAdapter = 1

OBJS_plexil_GlobalState = globals_adapter.o
LIBS_plexil_GlobalState = fawkescore \
                         PlexilAppFramework PlexilValue PlexilIntfc PlexilUtils pugixml
NOSOVER_plexil_GlobalState = 1

OBJS_all    = $(OBJS_plexil) $(OBJS_plexil_FawkesTimeAdapter) $(OBJS_plexil_FawkesLoggingAdapter) \
              $(OBJS_plexil_BehaviorEngineAdapter) $(OBJS_plexil_ThreadName) \
              $(OBJS_plexil_ProtobufCommAdapter) $(OBJS_plexil_NavGraphAdapter) \
              $(OBJS_plexil_FawkesRemoteAdapter) $(OBJS_plexil_FawkesConfigurationAdapter) \
              $(OBJS_plexil_GlobalState) $(OBJS_plexil_FawkesBlackboardAdapter)
LIBS_base   = $(LIBDIR)/plexil/FawkesTimeAdapter.$(SOEXT) \
              $(LIBDIR)/plexil/FawkesLoggingAdapter.$(SOEXT) \
              $(LIBDIR)/plexil/FawkesRemoteAdapter.$(SOEXT) \
              $(LIBDIR)/plexil/FawkesConfigurationAdapter.$(SOEXT) \
              $(LIBDIR)/plexil/FawkesBlackboardAdapter.$(SOEXT) \
              $(LIBDIR)/plexil/BehaviorEngineAdapter.$(SOEXT) \
              $(LIBDIR)/plexil/ThreadName.$(SOEXT) \
              $(LIBDIR)/plexil/GlobalState.$(SOEXT)
LIBS_pb     = $(LIBDIR)/plexil/ProtobufCommAdapter.$(SOEXT)
LIBS_navgraph = $(LIBDIR)/plexil/NavGraphAdapter.$(SOEXT)
LIBS_all    = $(LIBS_base) $(LIBS_pb) $(LIBS_navgraph)
PLUGINS_all = $(PLUGINDIR)/plexil.so


ifeq ($(HAVE_PLEXIL)$(HAVE_PLEXIL_BOOST_LIBS),11)
  CFLAGS  += $(CFLAGS_PLEXIL)  $(call boost-libs-cflags,$(PLEXIL_REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PLEXIL) $(call boost-libs-ldflags,$(PLEXIL_REQ_BOOST_LIBS))

	LIBS_build = $(LIBS_base)
  PLUGINS_build = $(PLUGINS_all)

  ifeq ($(HAVE_PROTOBUF)$(HAVE_PROTOBUF_COMM),11)
    LIBS_build += $(LIBS_pb)
  else
    ifneq ($(HAVE_PROTOBUF),1)
      WARN_TARGETS += warning_protobuf
    endif
    ifneq ($(HAVE_PROTOBUF_COMM),1)
      WARN_TARGETS += warning_protobuf_comm
    endif
  endif

  ifeq ($(HAVE_NAVGRAPH),1)
    LIBS_plexil += fawkesnavgraphaspect
    OBJS_plexil += navgraph_access_thread.o
    CFLAGS += -DHAVE_NAVGRAPH
    LIBS_build += $(LIBS_navgraph)
  else
    WARN_TARGETS += warning_navgraph
  endif

  ifeq ($(HAVE_CPP17),1)
    # Enforce C++14, as plexil is not compatible with ISO C++17.
    CFLAGS += -std=c++14
  endif

else
  ifneq ($(HAVE_PLEXIL),1)
    WARN_TARGETS += warning_plexil
  endif
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
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting NavGraphAdapter for plexil$(TNORMAL) (navgraphaspect not available)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ProtobufCommAdapter for plexil$(TNORMAL) (protobuf not available)"
warning_protobuf_comm:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ProtobufCommAdapter for plexil$(TNORMAL) (protobuf_comm not available)"
endif

include $(BUILDSYSDIR)/base.mk
