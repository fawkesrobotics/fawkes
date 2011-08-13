#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Tool
#                            -------------------
#   Created on Tue Sep 23 12:24:08 2008
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/ext/gmsl

CFLAGS  += $(CFLAGS_GUI)
LDFLAGS += $(LDFLAGS_GUI)

LIBS_ffconfiggui = stdc++ fawkescore fawkesconfig fawkesguiutils fvutils
OBJS_ffconfiggui = main.o			\
		   config_tree_view.o		\
		   config_edit_dialog.o		\
		   config_add_dialog.o		\
		   config_remove_dialog.o	\
		   config_editor.o			\
		   config_editor_plugin.o	\
		   retriever_config_plugin.o	\
		   naostiffness_config_plugin.o
OBJS_all = $(OBJS_ffconfiggui)

ifeq ($(HAVE_GUI),1)
  BINS_gui = $(BINDIR)/ffconfiggui
  INST_RESDIRS = guis/config_editor
  MANPAGES_gui = $(MANDIR)/man1/ffconfiggui.1
else
  WARN_TARGETS += warning_ffconfiggui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_ffconfiggui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building ffconfiggui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk

