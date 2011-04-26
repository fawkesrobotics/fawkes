
#*****************************************************************************
#            Makefile Build System for Fawkes: PanTilt Plugin
#                            -------------------
#   Created on Thu Dec 30 22:01:16 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_roomba = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface \
	      Roomba500Interface LedInterface SwitchInterface MotorInterface \
	      BatteryInterface
OBJS_roomba = roomba_plugin.o roomba_500.o thread_roomba_500.o sensor_thread.o

LIBS_roombajoy = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	         fawkesinterface Roomba500Interface JoystickInterface
OBJS_roombajoy = roombajoy_plugin.o roombajoy_thread.o

ifeq ($(HAVE_BLUEZ),1)
  CFLAGS         += $(CFLAGS_BLUEZ)
  LDFLAGS_roomba += $(LDFLAGS_BLUEZ)
else
  WARN_TARGETS += warning_bluez
endif

PLUGINS_all = $(PLUGINDIR)/roomba.so $(PLUGINDIR)/roombajoy.so

OBJS_all = $(OBJS_roomba) $(OBJS_roombajoy)

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BUILDSYSDIR)/interface.mk

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_bluez
warning_bluez:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting RooTooth support$(TNORMAL) (bluez-libs[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

