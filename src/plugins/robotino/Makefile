
#*****************************************************************************
#            Makefile Build System for Fawkes: Robotino Plugin
#                            -------------------
#   Created on Sun Nov 13 15:19:32 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

# Uncomment to enable simple time tracking
#CFLAGS += -DUSE_TIMETRACKER

LIBS_robotino = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface MotorInterface
OBJS_robotino = robotino_plugin.o sensor_thread.o act_thread.o

LIBS_robotino_joystick = m fawkescore fawkesutils fawkesaspects \
			 fawkesblackboard fawkesinterface \
			 JoystickInterface MotorInterface
OBJS_robotino_joystick = robotino_joystick_plugin.o joystick_thread.o

ifeq ($(HAVE_OPENROBOTINO),1)
  CFLAGS  += $(CFLAGS_OPENROBOTINO)
  LDFLAGS_robotino += $(LDFLAGS_OPENROBOTINO)

  PLUGINS_all = $(PLUGINDIR)/robotino.so $(PLUGINDIR)/robotino-joystick.so
else
  WARN_TARGETS += warning_openrobotino
endif

OBJS_all = $(OBJS_robotino) $(OBJS_robotino_joystick)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_openrobotino
warning_openrobotino:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino support$(TNORMAL) (OpenRobotino API not found)"
endif

include $(BUILDSYSDIR)/base.mk
