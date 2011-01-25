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

ifeq ($(HAVE_LIBMICROHTTPD),1)
  LIBS_webview = crypt fawkescore fawkesutils fawkesaspects fawkesblackboard \
		 fawkesinterface fawkeswebview fawkesplugin fawkesnetcomm
  OBJS_webview = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
  PLUGINS_all = $(PLUGINDIR)/webview.so
  INST_RESDIRS = webview
else
  WARN_TARGETS += warning_libmicrohttpd
endif

OBJS_all = $(OBJS_webview)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libmicrohttpd
warning_libmicrohttpd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting webview plugin$(TNORMAL) (libmicrohttpd not installed)"
endif

include $(BUILDSYSDIR)/base.mk

