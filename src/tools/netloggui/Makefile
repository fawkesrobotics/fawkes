#*****************************************************************************
#            Makefile Build System for Fawkes : NetLog GUI
#                            -------------------
#   Created on Tue Nov 04 23:35:43 2008
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

CFLAGS  += $(CFLAGS_GUI)
LDFLAGS += $(LDFLAGS_GUI)

LIBS_ffnetloggui = stdc++ fawkescore fawkesutils fawkesnetcomm fawkesguiutils
OBJS_ffnetloggui = main.o netloggui.o
OBJS_all = $(OBJS_ffnetloggui)

ifeq ($(HAVE_GUI),1)
  BINS_gui = $(BINDIR)/ffnetloggui
  INST_RESDIRS = guis/netloggui
  MANPAGES_gui = $(MANDIR)/man1/ffnetloggui.1
else
  WARN_TARGETS += warning_ffnetloggui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_ffnetloggui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building netloggui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk

