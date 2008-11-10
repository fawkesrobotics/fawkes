#*****************************************************************************
#            Makefile Build System for Fawkes : Plugin GUI
#                            -------------------
#   Created on Wed Nov 22 00:29:31 2006 (from plugin tool)
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

CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)

LIBS_ffplugingui = fawkescore fawkesutils fawkesnetcomm fawkesplugin fawkesguiutils
OBJS_ffplugingui = main.o plugin_gui.o plugin_tree_view.o
OBJS_all = $(OBJS_ffplugingui)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/ffplugingui
else
  WARN_TARGETS += warning_plugin_gui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_plugin_gui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building ffplugingui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

