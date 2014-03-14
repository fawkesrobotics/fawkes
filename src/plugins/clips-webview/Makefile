#*****************************************************************************
#          Makefile Build System for Fawkes: CLIPS Webview Plugin
#                            -------------------
#   Created on Sat Jun 15 19:51:04 2013
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_clips_webview = \
	fawkescore fawkesutils fawkesaspects fawkesclipsaspect \
	fawkeswebview
OBJS_clips_webview = clips-webview-plugin.o clips-webview-thread.o \
		     clips-webview-processor.o

OBJS_all = $(OBJS_clips_webview)

ifeq ($(HAVE_WEBVIEW)$(HAVE_CLIPS),11)
  CFLAGS  += $(CFLAGS_WEBVIEW)  $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_WEBVIEW) $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-webview.$(SOEXT)
else
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_clips
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-webview plugin$(TNORMAL) (webview not available)"

warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-webview plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
