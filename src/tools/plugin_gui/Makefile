#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Tool
#                            -------------------
#   Created on Wed Nov 22 00:29:31 2006
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
include $(BASEDIR)/etc/buildsys_local/gui.mk
include $(BASEDIR)/etc/buildsys/ext/gmsl

CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)

LIBS_pt_gui = core utils netcomm fawkesmainappmsgs
OBJS_pt_gui = main.o			\
	      plugin_gui.o		\
	      backend_thread.o

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  OBJS_all = $(OBJS_pt_gui)
  BINS_all = $(BINDIR)/pt_gui
else
  WARN_TARGETS += warning_plugin_gui
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_plugin_gui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building pt_gui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

