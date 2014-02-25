#*****************************************************************************
#          Makefile Build System for Fawkes: Webview PTZ Camera Plugin
#                            -------------------
#   Created on Fri Feb 07 15:58:05 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/webview/webview.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_webview_ptzcam = m \
	fawkescore fawkesutils fawkesaspects fawkeswebview fawkesblackboard \
	fawkesinterface PanTiltInterface CameraControlInterface SwitchInterface
OBJS_webview_ptzcam = webview-ptzcam-plugin.o webview-ptzcam-thread.o \
		     webview-ptzcam-processor.o

OBJS_all = $(OBJS_webview_ptzcam)

ifeq ($(HAVE_WEBVIEW),1)
  CFLAGS  += $(CFLAGS_WEBVIEW)
  LDFLAGS += $(LDFLAGS_WEBVIEW)

  PLUGINS_all = $(PLUGINDIR)/webview-ptzcam.$(SOEXT)
else
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_webview
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build webview-ptzcam plugin$(TNORMAL) (webview not available)"
endif

include $(BUILDSYSDIR)/base.mk
