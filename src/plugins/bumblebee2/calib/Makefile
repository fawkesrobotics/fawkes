#*****************************************************************************
#        Makefile Build System for Fawkes : Bumblebee2 calibration GUI
#                            -------------------
#   Created on Thu Jul 18 20:34:06 2013
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

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/gui.mk

CFLAGS  += $(CFLAGS_GUI)
LDFLAGS += $(LDFLAGS_GUI)

LIBS_ffbb2calib = fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		  fawkesguiutils fawkesinterface fvutils fvcams \
		  OpenCVStereoParamsInterface
OBJS_ffbb2calib = main.o bb2calib.o
OBJS_all = $(OBJS_ffbb2calib)

ifeq ($(HAVE_GTKMM),1)
  BINS_gui = $(BINDIR)/ffbb2calib
else
  WARN_TARGETS += warning_gui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_gui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building ffbb2calib$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk
