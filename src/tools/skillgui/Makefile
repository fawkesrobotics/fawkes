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
include $(BUILDSYSDIR)/utils.mk

# Set to 1 to use Pyprus instead of Cairo for rendering the graph
USE_PAPYRUS=0

ifneq ($(PKGCONFIG),)
  HAVE_GRAPHVIZ = $(if $(shell $(PKGCONFIG) --exists 'libgvc'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_GRAPHVIZ),1)
    _GRAPHVIZ_VERSION_OK = $(if $(shell $(PKGCONFIG) --atleast-version=2.22 'libgvc'; echo $${?/1/}),1,0)
    GRAPHVIZ_ATLEAST_230 = $(if $(shell $(PKGCONFIG) --atleast-version=2.30 'libgvc'; echo $${?/1/}),1,0)
    GRAPHVIZ_HAVE_GRAPH  = $(if $(shell $(PKGCONFIG) --exists 'libgraph'; echo $${?/1/}),1,0)
    ifneq ($(_GRAPHVIZ_VERSION_OK),1)
      HAVE_GRAPHVIZ=0
      GUI_ERROR += (Graphviz too old, need at least version 2.22)
    endif
  else
    GUI_ERROR += (graphviz not installed)
  endif

  ifeq ($(USE_PAPYRUS),1)
    HAVE_PAPYRUS  = $(if $(shell $(PKGCONFIG) --exists 'papyrus-1.0' 'papyrus-gtkmm-1.0'; echo $${?/1/}),1,0)
    ifeq ($(HAVE_PAPYRUS),1)
      CFLAGS_PAPYRUS  = $(shell $(PKGCONFIG) --cflags 'papyrus-1.0' 'papyrus-gtkmm-1.0')
      LDFLAGS_PAPYRUS = $(shell $(PKGCONFIG) --libs 'papyrus-1.0' 'papyrus-gtkmm-1.0')
    else
      GUI_ERROR += (papyrus/papyrusmm not installed)
    endif
  endif
  ifeq ($(HAVE_GRAPHVIZ),1)
    CFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --cflags 'libgvc') -DWITH_CODEGENS
    ifeq ($(GRAPHVIZ_HAVE_GRAPH),1)
      LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libgvc')
    else
      # "--libs 'libgvc'" requires -lgraph, even if only cgraph is installed. Not fixed in stable releases yet
      LDFLAGS_GRAPHVIZ = $(shell $(PKGCONFIG) --libs 'libcgraph') -lgvc
      # add WITH_CGRAPH, in case something is wrong with libgvc. Should be provided by "/usr/include/graphviz/types.h"
      CFLAGS_GRAPHVIZ += -DWITH_CGRAPH
    endif
    ifeq ($(GRAPHVIZ_ATLEAST_230),1)
      CFLAGS_GRAPHVIZ += -DGRAPHVIZ_ATLEAST_230
    endif
    VERSION_GRAPHVIZ = $(shell $(PKGCONFIG) --modversion 'libgvc')
    CFLAGS_GRAPHVIZ += -DGRAPHVIZ_VERSION=$(call version_number,$(VERSION_GRAPHVIZ))
  endif
endif

CFLAGS  += $(CFLAGS_GUI) $(CFLAGS_GRAPHVIZ) $(CFLAGS_PAPYRUS)
LDFLAGS += $(LDFLAGS_GUI) $(LDFLAGS_GRAPHVIZ) $(LDFLAGS_PAPYRUS)

LIBS_skillgui = stdc++ m fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		fawkesguiutils fawkesinterface \
		SkillerInterface SkillerDebugInterface
OBJS_skillgui = main.o skillgui.o

ifeq ($(USE_PAPYRUS),1)
  CFLAGS  += $(CFLAGS_PAPYRUS) -std=gnu++0x -DUSE_PAPYRUS
  LDFLAGS += $(LDFLAGS_PAPYRUS)
  OBJS_skillgui += graph_viewport.o gvplugin_skillgui_papyrus.o
  PAPYRUS_OK = $(HAVE_PAPYRUS)
else
  PAPYRUS_OK = 1
  OBJS_skillgui += graph_drawing_area.o gvplugin_skillgui_cairo.o
endif

LIBS_skillgui_batch_render = stdc++ m fawkescore fawkesutils
OBJS_skillgui_batch_render = batch_render.o gvplugin_skillgui_cairo.o

OBJS_all = $(OBJS_skillgui) $(OBJS_skillgui_batch_render)

ifeq ($(HAVE_GUI)$(HAVE_GRAPHVIZ),11)
  BINS_gui = $(BINDIR)/skillgui $(BINDIR)/skillgui_batch_render
  INST_RESDIRS = guis/skillgui
  MANPAGES_all = $(MANDIR)/man1/skillgui.1 $(MANDIR)/man1/skillgui_batch_render.1
else
  WARN_TARGETS += warning_skillgui
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
gui: $(WARN_TARGETS)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_skillgui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building skillgui$(TNORMAL) $(GUI_ERROR)";

  endif
endif

include $(BUILDSYSDIR)/base.mk

