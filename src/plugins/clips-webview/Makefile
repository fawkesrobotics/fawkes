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

LIBS_clips_webview = \
	fawkescore fawkesutils fawkesaspects fawkesclipsaspect \
	fawkeswebview
OBJS_clips_webview = clips-webview-plugin.o clips-webview-thread.o \
		     clips-webview-processor.o

OBJS_all = $(OBJS_clips_webview)

ifeq ($(HAVE_CPP11)$(HAVE_WEBVIEW)$(HAVE_CLIPS),111)
  CFLAGS  += $(CFLAGS_WEBVIEW)  $(CFLAGS_CLIPS) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_WEBVIEW) $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-webview.$(SOEXT)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_clips warning_webview warning_cpp11
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-webview plugin$(TNORMAL) (webview not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-webview plugin$(TNORMAL) (C++11 not supported)"
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-webview plugin$(TNORMAL) (CLIPS not found)"
endif

include $(BUILDSYSDIR)/base.mk
