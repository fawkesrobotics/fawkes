#*****************************************************************************
#            Makefile Build System for Fawkes : World Info Viewer
#                            -------------------
#   Created on Wed April 09 20:17:52 2007
#   Copyright (C) 2008 by Daniel Beck, AllemaniACs RoboCup Team
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

LIBS_wiv = fawkesblackboard fawkesconfig fawkescore fawkesgeometry fawkesinterfaces fawkesnetcomm fawkesutils fawkesworldinfo_utils
OBJS_wiv = main.o			\
	   worldinfo_viewer.o		\
	   field_view.o			\
	   backend_thread.o

OBJS_all = $(OBJS_wiv)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/wiv
else
  WARN_TARGETS += warning_wiv
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_wiv:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building wiv$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

