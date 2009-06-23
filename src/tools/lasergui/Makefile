#*****************************************************************************
#          Makefile Build System for Fawkes : Laser Visualization
#                            -------------------
#   Created on Thu Oct 09 13:05:34 2008
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


LIBS_lasergui = fawkescore fawkesblackboard fawkesguiutils \
		Laser360Interface Laser720Interface
ifeq ($(BUILD_TYPE),maemo)
  ifeq ($(HAVE_HILDONMM),1)
    OBJS_lasergui = lasergui_hildon.o
    CFLAGS  += $(CFLAGS_HILDONMM)
    LDFLAGS += $(LDFLAGS_HILDONMM)
  else
    WARN_TARGETS += warning_hildon
  endif
else
  OBJS_lasergui = lasergui.o
endif
OBJS_lasergui += laser_drawing_area.o
OBJS_all = $(OBJS_lasergui)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui = $(BINDIR)/lasergui
else
  WARN_TARGETS += warning_lasergui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_lasergui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building lasergui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

