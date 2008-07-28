#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Tool
#                            -------------------
#   Created on Wed Nov 22 00:29:31 2006
#   Copyright (C) 2007 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/gui.mk

CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)

LIBS_nlm = fawkescore fawkesutils fawkesnetcomm fawkesmainappmsgs
OBJS_nlm = main.o			\
	      netlog_monitor.o		\
	      backend_thread.o
OBJS_all = $(OBJS_nlm)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/nlm
else
  WARN_TARGETS += warning_nlm
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_nlm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building nlm$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

