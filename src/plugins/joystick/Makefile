#*****************************************************************************
#           Makefile Build System for Fawkes: Joystick Plugin
#                            -------------------
#   Created on Sat Nov 22 18:02:00 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_joystick = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		fawkesinterface JoystickInterface
OBJS_joystick = joystick_plugin.o sensor_thread.o acquisition_thread.o \
		force_feedback.o act_thread.o

LIBS_ffjoystick = stdc++ fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface JoystickInterface
OBJS_ffjoystick = ffjoystick.o acquisition_thread.o remote_bb_poster.o \
		  force_feedback.o

LIBS_ffkbjoystick = stdc++ fawkescore fawkesutils fawkesaspects fawkesblackboard \
		    fawkesinterface JoystickInterface
OBJS_ffkbjoystick = ffkbjoystick.o acquisition_thread.o remote_bb_poster.o \
		    force_feedback.o

OBJS_all = $(OBJS_joystick) $(OBJS_ffjoystick) $(OBJS_ffkbjoystick)

BINS_all     += $(BINDIR)/ffkbjoystick
MANPAGES_all += $(MANDIR)/man1/ffkbjoystick.1

ifeq ($(OS),Linux)
  BINS_all     += $(BINDIR)/ffjoystick
  PLUGINS_all  += $(PLUGINDIR)/joystick.so
  MANPAGES_all += $(MANDIR)/man1/ffjoystick.1
else
  WARN_TARGETS += warning_onlylinux
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_onlylinux
warning_onlylinux:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Joystick Plugin$(TNORMAL) (currently only Linux is supported)"
endif

include $(BUILDSYSDIR)/base.mk
