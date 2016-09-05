#*****************************************************************************
#            Makefile Build System for Fawkes : Skill GUI
#                            -------------------
#   Created on Mon Nov 03 13:32:18 2008
#   Copyright (C) 2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/gui.mk


ifneq ($(PKGCONFIG),)
  HAVE_GRAPHVIZ = $(if $(shell $(PKGCONFIG) --exists 'libgvc'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_GRAPHVIZ),1)
    _GRAPHVIZ_VERSION_OK = $(if $(shell $(PKGCONFIG) --atleast-version=2.22 'libgvc'; echo $${?/1/}),1,0)
    ifneq ($(_GRAPHVIZ_VERSION_OK),1)
      HAVE_GRAPHVIZ=0
      GUI_ERROR += (Graphviz too old, need at least version 2.22)
    endif
  else
    GUI_ERROR += (graphviz not installed)
  endif

  
  ifeq ($(HAVE_GRAPHVIZ),1)
    CFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --cflags 'libgvc') -DWITH_CODEGENS
    LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libgvc')
  endif
endif

CFLAGS  += $(CFLAGS_GUI) $(CFLAGS_GRAPHVIZ)
LDFLAGS += $(LDFLAGS_GUI) $(LDFLAGS_GRAPHVIZ)

LIBS_ffeclipse_debugger = stdc++ m fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		fawkesguiutils fawkesinterface EclipseDebuggerInterface
OBJS_ffeclipse_debugger = main.o eclipse_debugger.o

OBJS_all = $(OBJS_ffeclipse_debugger)

ifeq ($(HAVE_GUI),1)
  BINS_gui = $(BINDIR)/ffeclipse-debugger 
else
  WARN_TARGETS += warning_eclipsedebugger
endif

INST_RESDIRS = guis/eclipsedebugger


ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
gui: $(WARN_TARGETS)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_eclipsedebugger:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building eclipsedbugger$(TNORMAL) $(GUI_ERROR)";

  endif
endif

include $(BUILDSYSDIR)/base.mk

