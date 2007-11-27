#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin Tool
#                            -------------------
#   Created on Wed Nov 22 00:29:31 2006
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys_local/gtkconf.mk

CFLAGS = -g
CFLAGS += $(GTK_CFLAGS)
LDFLAGS += $(GTK_LDFLAGS)

LIBS_pt_gui = core utils netcomm fawkesmainappmsgs
OBJS_pt_gui = main.o			\
	      plugin_gui.o		\
	      backend_thread.o

ifeq ($(HAVE_GTKMM-2.4)$(HAVE_LIBGLADEMM-2.4),11)
  OBJS_all = $(OBJS_pt_gui)
  BINS_all = $(BINDIR)/pt_gui
else
  WARN_TARGETS += warning_plugin_gui
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_plugin_gui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building pt_gui$(TNORMAL) ($(GTK_CONF_ERR) not installed)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

