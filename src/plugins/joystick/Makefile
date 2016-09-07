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

LIBS_joystick = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		fawkesinterface JoystickInterface
OBJS_joystick = joystick_plugin.o sensor_thread.o acquisition_thread.o \
		bb_handler.o force_feedback.o act_thread.o

LIBS_ffjoystick = stdc++ m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkeslogging JoystickInterface
OBJS_ffjoystick = ffjoystick.o acquisition_thread.o remote_bb_poster.o \
		  bb_handler.o force_feedback.o act_thread.o

LIBS_ffkbjoystick = stdc++ m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		    fawkesinterface fawkeslogging JoystickInterface
OBJS_ffkbjoystick = ffkbjoystick.o remote_bb_poster.o bb_handler.o

LIBS_joystick_teleop = m fawkescore fawkesutils fawkesaspects \
		       fawkesblackboard fawkesinterface \
		       JoystickInterface MotorInterface Laser360Interface
OBJS_joystick_teleop = joystick_teleop_plugin.o joystick_teleop_thread.o

OBJS_all = $(OBJS_joystick) $(OBJS_ffjoystick) $(OBJS_ffkbjoystick) $(OBJS_joystick_teleop) 

BINS_all     += $(BINDIR)/ffkbjoystick
PLUGINS_all  += $(PLUGINDIR)/joystick-teleop.so
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
