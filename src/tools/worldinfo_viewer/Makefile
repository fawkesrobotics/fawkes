#*****************************************************************************
#            Makefile Build System for Fawkes : World Info Viewer
#                            -------------------
#   Created on Wed April 09 20:17:52 2007
#   Copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
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

# enable for debug output
#CFLAGS += -DDEBUG_PRINT

LIBS_ffwiviewer = stdc++ fawkesblackboard fawkesconfig fawkescore fawkesnetcomm fawkesutils fawkesworldinfo_utils fawkesgeometry
OBJS_ffwiviewer = main.o			\
		  worldinfo_viewer.o		\
		  field_view.o			\
		  backend_thread.o

OBJS_all = $(OBJS_ffwiviewer)

ifeq ($(HAVE_GUI),1)
  BINS_gui = $(BINDIR)/ffwiviewer
  INST_RESDIRS = guis/worldinfo_viewer
  MANPAGES_gui = $(MANDIR)/man1/ffwiviewer.1
else
  WARN_TARGETS += warning_wiv
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_wiv:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Not building ffwiviewer$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk

