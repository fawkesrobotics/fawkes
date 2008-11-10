#*****************************************************************************
#            Makefile Build System for Fawkes : Skill GUI
#                            -------------------
#   Created on Mon Nov 03 13:32:18 2008
#   Copyright (C) 2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)

LIBS_skillgui = fawkescore fawkesutils fawkesnetcomm fawkesblackboard fawkesinterfaces fawkesguiutils
OBJS_skillgui = main.o skillgui.o
OBJS_all = $(OBJS_skillgui)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/skillgui
else
  WARN_TARGETS += warning_skillgui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_skillgui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building skillgui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

