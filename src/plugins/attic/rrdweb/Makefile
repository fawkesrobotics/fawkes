#*****************************************************************************
#           Makefile Build System for Fawkes: RRD Webview Plugin
#                            -------------------
#   Created on Tue Dec 21 00:59:31 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/plugins/rrd/rrd.mk
include $(BUILDCONFDIR)/webview/webview.mk

LIBS_rrdweb = fawkescore fawkesaspects fawkesutils fawkesrrdaspect fawkeswebview
OBJS_rrdweb = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_rrdweb)

ifeq ($(HAVE_RRD)$(HAVE_WEBVIEW),11)
  PLUGINS_all = $(PLUGINDIR)/rrdweb.so
else
  ifneq ($(HAVE_RRD),1)
    WARN_TARGETS += warning_rrd
  endif
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_rrd warning webview
warning_rrd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RRD Webview Plugin$(TNORMAL) (rrd plugin not available)"

warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RRD Webview Plugin$(TNORMAL) (webview not available)"
endif

include $(BUILDSYSDIR)/base.mk
