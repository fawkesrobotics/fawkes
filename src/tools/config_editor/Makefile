#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Tool
#                            -------------------
#   Created on Tue Sep 23 12:24:08 2008
#   Copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
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
include $(BASEDIR)/etc/buildsys/gui.mk
include $(BASEDIR)/etc/buildsys/ext/gmsl

CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)

LIBS_ce = fawkesconfig fawkesguiutils
OBJS_ce = main.o			\
	  config_tree_view.o		\
	  config_edit_dialog.o		\
	  config_add_dialog.o		\
	  config_remove_dialog.o	\
	  config_editor.o	
OBJS_all = $(OBJS_ce)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/ce
else
  WARN_TARGETS += warning_ce
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_ce:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building ce$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk
