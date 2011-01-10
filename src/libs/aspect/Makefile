#*****************************************************************************
#                 Makefile Build System for Fawkes: Aspects
#                            -------------------
#   Created on Thu Jan 11 16:41:34 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libfawkesaspects = fawkescore fawkesutils fawkesblackboard fawkesnetcomm fawkesconfig fawkesplugin

ifeq ($(HAVE_FIREVISION),1)
  INCDIRS += $(realpath $(BASEDIR)/src/firevision)
else
  OBJS_FILTER_OUT += $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/vision*.cpp $(SRCDIR)/inifins/vision*.cpp))
  WARN_TARGETS += warning_vision
endif

ifneq ($(HAVE_WEBVIEW),1)
  OBJS_FILTER_OUT += $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/webview.cpp $(SRCDIR)/inifins/webview.cpp))
  WARN_TARGETS += warning_webview
else
  CFLAGS += -DHAVE_WEBVIEW
  LIBS_libfawkesaspects += fawkeswebview
endif

OBJS_libfawkesaspects = $(patsubst %.cpp,%.o,$(filter-out $(OBJS_FILTER_OUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))))
HDRS_libfawkesaspects = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h))
INST_HDRS_SUBDIR_libfawkesaspects = aspect

OBJS_all = $(OBJS_libfawkesaspects)
LIBS_all  = $(LIBDIR)/libfawkesaspects.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_webview warning_vision
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Excluding WebviewAspect$(TNORMAL) (libfawkeswebview not built)"

warning_vision:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Excluding vision aspects$(TNORMAL) (FireVision not available)"

endif

include $(BUILDSYSDIR)/base.mk

