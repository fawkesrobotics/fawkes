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

LIBS_joystick = fawkescore fawkesutils fawkesaspects JoystickInterface
OBJS_joystick = joystick_plugin.o sensor_thread.o acquisition_thread.o

LIBS_ffjoystick = fawkescore fawkesutils fawkesaspects JoystickInterface
OBJS_ffjoystick = ffjoystick.o acquisition_thread.o

OBJS_all = $(OBJS_joystick) $(OBJS_ffjoystick)

ifeq ($(OS),Linux)
  BINS_all = $(BINDIR)/ffjoystick
  PLUGINS_all = $(PLUGINDIR)/joystick.so
else
  WARN_TARGETS += warning_onlylinux
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_onlylinux
warning_onlylinux:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Joystick Plugin$(TNORMAL) (currently only Linux is supported)"
endif

include $(BASEDIR)/etc/buildsys/base.mk
