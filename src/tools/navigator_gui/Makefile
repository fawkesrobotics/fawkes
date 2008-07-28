#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator GUI
#                            -------------------
#   Created on Tue May 29 20:00:00 2007
#   Copyright (C) 2007 by Martin Liebenberg, AllemaniACs RoboCup Team
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
-include $(BASEDIR)/src/plugins/navigator/navigator.mk

CFLAGS  += $(CFLAGS_GTKMM)
LDFLAGS += $(LDFLAGS_GTKMM)

LIBS_navigator_gui = fawkescore fawkesutils fawkesnetcomm navigator fawkesmainappmsgs
OBJS_navigator_gui = navigator_gui.o

OBJS_all = $(OBJS_navigator_gui)

ifeq ($(BUILD_NAVIGATOR)$(HAVE_GTKMM),11)
  BINS_gui = $(BINDIR)/navigator_gui
else
  WARN_TARGETS += warning_nonavi
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)
.PHONY: warning_nonavi
warning_nonavi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Navigator GUI (navigator not built)$(TNORMAL) $(NAVIGATOR_ERROR)$(GUI_ERROR)"
endif

include $(BASEDIR)/etc/buildsys/base.mk

