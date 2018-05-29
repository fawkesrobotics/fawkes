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
include $(BUILDSYSDIR)/boost.mk

ifneq ($(PKGCONFIG),)
  HAVE_APR_UTIL = $(if $(shell $(PKGCONFIG) --exists 'apr-util-1'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_APR_UTIL),1)
    CFLAGS_APR_UTIL  = -DHAVE_APR_UTIL $(shell $(PKGCONFIG) --cflags 'apr-util-1')
    LDFLAGS_APR_UTIL = $(shell $(PKGCONFIG) --libs 'apr-util-1')
  endif
endif

REQ_BOOST_LIBS = filesystem
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))


ifeq ($(HAVE_BOOST_LIBS)$(HAVE_LIBMICROHTTPD),11)
  LIBS_webview = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
     fawkesinterface fawkeswebview fawkesplugin fawkesnetcomm \
     fawkeslogging
  OBJS_webview = webview_plugin.o webview_thread.o \
                 service_browse_handler.o user_verifier.o \
                 static_processor.o rest_processor.o
  PLUGINS_all = $(PLUGINDIR)/webview.so
  INST_RESDIRS = webview

  CFLAGS  += $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

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
                    backendinfo-rest-api/backendinfo-rest-api.o \
                    plugin-rest-api/plugin-rest-api.o \
                    config-rest-api/config-rest-api.o \
                   $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*-rest-api/model/*.cpp))))

    ifeq ($(HAVE_TF),1)
      OBJS_webview += tf-rest-api/tf-rest-api.o
      LIBS_webview += fawkestf
      CFLAGS  += $(CFLAGS_TF)
      LDFLAGS += $(LDFLAGS_TF)
    else
      WARN_TARGETS += warning_tf
    endif

    ifeq ($(HAVE_JPEG),1)
      CFLAGS += -DHAVE_JPEG
      LIBS_webview += m fvutils fvcams
      OBJS_webview += image-rest-api/image-rest-api.o \
                      image-rest-api/mjpeg_reply.o \
                      image-rest-api/jpeg_stream_producer.o
    else
      WARN_TARGETS = warning_jpeg
    endif

  else
    WARN_TARGETS += warning_rapidjson
  endif

  OBJS_all = $(OBJS_webview)

else
  ifneq ($(HAVE_LIBMICROHTTPD),1)
    WARN_TARGETS += warning_libmicrohttpd
  endif
  ifneq ($(HAVE_CPP17),1)
    WARN_TARGETS += warning_cpp17
  endif
  ifneq ($(HAVE_RAPIDJSON),1)
    WARN_TARGETS += warning_rapidjson
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libmicrohttpd warning_tf warning_jpeg warning_apr_util warning_cpp11 warning_cpp17 warning_rapidjson $(WARN_TARGETS_BOOST)

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

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview plugin$(TNORMAL) (Boost library $* not found)"

endif

include $(BUILDSYSDIR)/base.mk

