#*****************************************************************************
#            Makefile Build System for Fawkes: Webview Plugin
#                            -------------------
#   Created on Mon Oct 13 17:57:04 2008 (I5 Developer's Day)
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/download.mk
include $(LIBSRCDIR)/webview/webview.mk
include $(BUILDCONFDIR)/tf/tf.mk

ifneq ($(PKGCONFIG),)
  HAVE_GRAPHVIZ = $(if $(shell $(PKGCONFIG) --exists 'libgvc'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_GRAPHVIZ),1)
    GRAPHVIZ_HAVE_GRAPH  = $(if $(shell $(PKGCONFIG) --exists 'libgraph'; echo $${?/1/}),1,0)

    CFLAGS_GRAPHVIZ = -DHAVE_GRAPHVIZ $(shell $(PKGCONFIG) --cflags 'libgvc') -DWITH_CODEGENS
    ifeq ($(GRAPHVIZ_HAVE_GRAPH),1)
      LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libgvc')
    else
      # "--libs 'libgvc'" requires -lgraph, even if only cgraph is installed.
      # Not fixed in stable releases yet
      LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libcgraph') -lgvc
      # add WITH_CGRAPH, in case something is wrong with libgvc.
      # Should be provided by "/usr/include/graphviz/types.h"
      CFLAGS_GRAPHVIZ += -DWITH_CGRAPH
    endif
  endif
endif

HAVE_TF=0

ifeq ($(HAVE_LIBMICROHTTPD),1)
  LIBS_webview = crypt fawkescore fawkesutils fawkesaspects fawkesblackboard \
		 fawkesinterface fawkeswebview fawkesplugin fawkesnetcomm \
		 fawkeslogging
  OBJS_webview = blackboard_processor.o footer_generator.o header_generator.o \
		 service_browse_handler.o startpage_processor.o static_processor.o \
		 plugins_processor.o user_verifier.o webview_thread.o webview_plugin.o
  PLUGINS_all = $(PLUGINDIR)/webview.so
  INST_RESDIRS = webview
  ifeq ($(HAVE_TF)$(HAVE_GRAPHVIZ),11)
    OBJS_webview += tf_processor.o
    LIBS_webview += fawkestf
    CFLAGS  += $(CFLAGS_TF) $(CFLAGS_GRAPHVIZ)
    LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_GRAPHVIZ)
  else
    ifneq ($(HAVE_TF),1)
      WARN_TARGETS += warning_tf
    endif
    ifneq ($(HAVE_GRAPHVIZ),1)
      WARN_TARGETS += warning_graphviz
    endif
  endif
else
  WARN_TARGETS += warning_libmicrohttpd
endif

OBJS_all = $(OBJS_webview)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libmicrohttpd warning_tf warning_graphviz
warning_libmicrohttpd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting webview plugin$(TNORMAL) (libmicrohttpd not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms support$(TNORMAL) (TF not available)"
warning_graphviz:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms support$(TNORMAL) (graphviz not found)"
endif

include $(BUILDSYSDIR)/base.mk

