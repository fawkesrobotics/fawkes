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

CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)

LIBS_ffbatterymon = stdc++ fawkescore fawkesconfig fawkesnetcomm \
		    fawkesguiutils fawkesblackboard fawkesinterface \
		    fvutils BatteryInterface
OBJS_ffbatterymon = 	main.o				\
			battery_monitor_treeview.o	\
			battery_monitor.o
OBJS_all = $(OBJS_ffbatterymon)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/ffbatterymon
  INST_RESDIRS = guis/battery_monitor
else
  WARN_TARGETS += warning_ffbatterymon
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_ffbatterymon:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building ffbatterymon$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk
