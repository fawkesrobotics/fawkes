#*****************************************************************************
#                 Makefile Build System for Fawkes : Nao GUI
#                            -------------------
#   Created on Mon Apr 14 13:01:17 2008
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

LIBS_ffnaogui = fawkescore fawkesutils fawkesnetcomm fawkesblackboard \
		fawkesguiutils fawkesinterface \
	        NaoJointPositionInterface NaoJointStiffnessInterface \
		NaoSensorInterface SpeechSynthInterface LedInterface \
		HumanoidMotionInterface NavigatorInterface SwitchInterface m
OBJS_ffnaogui = main.o naogui.o
OBJS_all = $(OBJS_ffnaogui)

ifeq ($(HAVE_GTKMM),1)
  BINS_gui = $(BINDIR)/ffnaogui
else
  WARN_TARGETS += warning_naogui
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_naogui:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building naogui$(TNORMAL) $(GUI_ERROR)";
endif

include $(BUILDSYSDIR)/base.mk
