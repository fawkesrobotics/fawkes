#*****************************************************************************
#     Makefile Build System for Fawkes: metrics publishing
#                            -------------------
#   Created on Sat May 06 19:17:11 2017
#   Copyright (C) 2017 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/webview/webview.mk
include $(BUILDSYSDIR)/protobuf.mk


PRESUBDIRS = aspect interfaces protobuf

LIBS_metrics = fawkescore fawkesutils fawkesaspects	\
  fawkesinterface fawkesblackboard fawkeswebview fawkesmetricsaspect \
  MetricFamilyInterface MetricCounterInterface MetricGaugeInterface \
  MetricHistogramInterface MetricUntypedInterface \
	metrics_msgs

OBJS_metrics = metrics_plugin.o metrics_thread.o metrics_processor.o
OBJS_all    = $(OBJS_metrics)

ifeq ($(HAVE_WEBVIEW)$(HAVE_CPP11)$(HAVE_PROTOBUF),111)
  PLUGINS_all = $(PLUGINDIR)/metrics.$(SOEXT)
  CFLAGS  += $(CFLAGS_CPP11) $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_PROTOBUF)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS = warning_cpp11
  endif
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
endif

aspect: | protobuf

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp11 warning_webview warning_protobuf
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting metrics plugin$(TNORMAL) (C++11 support required)"
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting metrics plugin$(TNORMAL) (webview not available)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting metrics plugin$(TNORMAL) (protobuf not available)"
endif

include $(BUILDSYSDIR)/base.mk
