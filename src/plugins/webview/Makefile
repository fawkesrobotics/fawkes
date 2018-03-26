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
include $(BUILDSYSDIR)/utils.mk
include $(LIBSRCDIR)/webview/webview.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/fvconf.mk

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

    VERSION_GRAPHVIZ = $(shell $(PKGCONFIG) --modversion 'libgvc')
    CFLAGS_GRAPHVIZ += -DGRAPHVIZ_VERSION=$(call version_number,$(VERSION_GRAPHVIZ))
  endif

  HAVE_APR_UTIL = $(if $(shell $(PKGCONFIG) --exists 'apr-util-1'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_APR_UTIL),1)
    CFLAGS_APR_UTIL  = -DHAVE_APR_UTIL $(shell $(PKGCONFIG) --cflags 'apr-util-1')
    LDFLAGS_APR_UTIL = $(shell $(PKGCONFIG) --libs 'apr-util-1')
  endif
endif

ifeq ($(HAVE_LIBMICROHTTPD),1)
  LIBS_webview = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		 fawkesinterface fawkeswebview fawkesplugin fawkesnetcomm \
		 fawkeslogging
  OBJS_webview = blackboard_processor.o footer_generator.o header_generator.o \
		 service_browse_handler.o startpage_processor.o static_processor.o \
		 plugins_processor.o user_verifier.o webview_thread.o webview_plugin.o \
     rest_processor.o
  PLUGINS_all = $(PLUGINDIR)/webview.so
  INST_RESDIRS = webview

  ifeq ($(HAVE_CPP11)$(HAVE_JPEG),11)
    LIBS_webview += m fvutils fvcams
    OBJS_webview += image_processor.o mjpeg_reply.o jpeg_stream_producer.o
    CFLAGS += -DHAVE_JPEG $(CFLAGS_CPP11)
  else
    ifneq ($(HAVE_CPP11),1)
      WARN_TARGETS += warning_cpp11
    endif
    ifneq ($(HAVE_JPEG),1)
      WARN_TARGETS = warning_jpeg
    endif
  endif

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

  ifeq ($(HAVE_APR_UTIL),1)
    CFLAGS  += $(CFLAGS_APR_UTIL)
    LDFLAGS += $(LDFLAGS_APR_UTIL)
  else
    CFLAGS  += -DHAVE_CRYPT
    LIBS_webview += crypt
    WARN_TARGETS += warning_apr_util
  endif

  ifeq ($(HAVE_CPP17)$(HAVE_RAPIDJSON),11)
    CFLAGS  += -DHAVE_REST_APIS $(CFLAGS_CPP17)  $(CFLAGS_RAPIDJSON)
    LDFLAGS += $(LDFLAGS_CPP17) $(LDFLAGS_RAPIDJSON)

    OBJS_webview += blackboard-rest-api/blackboard-rest-api.o \
                   $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/blackboard-rest-api/model/*.cpp))))
  else
    ifneq ($(HAVE_CPP17),1)
      WARN_TARGETS += warning_cpp17
    endif
    ifneq ($(HAVE_RAPIDJSON),1)
      WARN_TARGETS += warning_rapidjson
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
.PHONY: warning_libmicrohttpd warning_tf warning_graphviz warning_jpeg warning_apr_util warning_cpp11 warning_cpp17 warning_rapidjson
warning_libmicrohttpd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting webview plugin$(TNORMAL) (libmicrohttpd not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms support$(TNORMAL) (TF not available)"
warning_graphviz:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms support$(TNORMAL) (graphviz not found)"
warning_jpeg:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting image viewing support$(TNORMAL) (JPEG not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting image viewing support$(TNORMAL) (C++11 not supported)"
warning_apr_util:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting APR based password verification support$(TNORMAL) (apr-util not found)"
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting REST APIs$(TNORMAL) (C++17 not supported)"
warning_rapidjson:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting REST APIs$(TNORMAL) (RapidJSON not available)"
endif

include $(BUILDSYSDIR)/base.mk

