#*****************************************************************************
#            Makefile Build System for Fawkes: XmlRpc Plugin
#                            -------------------
#   Created on Sun Aug 30 12:34:34 2009 (NRW Council Election)
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifeq ($(HAVE_LIBMICROHTTPD),1)
  LDFLAGS += $(LDFLAGS_LIBMICROHTTPD)
  CFLAGS  += $(CFLAGS_LIBMICROHTTPD)
else
  WARN_TARGETS += warning_libmicrohttpd
endif

ifeq ($(HAVE_LIBXMLRPCPP),1)
  LDFLAGS += $(LDFLAGS_LIBXMLRPCPP)
  CFLAGS  += $(CFLAGS_LIBXMLRPCPP)
else
  WARN_TARGETS += warning_libxmlrpcpp
endif

ifeq ($(HAVE_LIBMICROHTTPD)$(HAVE_LIBXMLRPCPP),11)
  LIBS_xmlrpc = fawkescore fawkesutils fawkesaspects fawkeswebview fawkesnetcomm \
		fawkesplugin fawkeslogging
  OBJS_xmlrpc = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))
  PLUGINS_all = $(PLUGINDIR)/xmlrpc.so
endif

OBJS_all = $(OBJS_xmlrpc)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libmicrohttpd warning_libxmlrpcpp
warning_libmicrohttpd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting xmlrpc plugin$(TNORMAL) (libmicrohttpd not installed)"
warning_libxmlrpcpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting xmlrpc plugin$(TNORMAL) (xmlrpc-c-c++ not installed)"
endif

include $(BUILDSYSDIR)/base.mk

