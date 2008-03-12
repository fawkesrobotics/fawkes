#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator Plugin
#                            -------------------
#   Created on Tue May 29 18:41:17 2007
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

ifneq ($(HAVE_VMC),1)
  WARN_TARGETS += warning_vmc
endif
ifneq ($(HAVE_GTS),1)
  WARN_TARGETS += warning_gts
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_vmc
warning_vmc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Navigator Plugin$(TNORMAL) (install libvmc)"

.PHONY: warning_gts
warning_gts:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Navigator Plugin$(TNORMAL) (install GTS)"
endif

ifeq ($(BUILD_NAVIGATOR),1)
  PLUGINS_all = $(PLUGINDIR)/navigator.so
  CFLAGS  = $(shell $(PKGCONFIG) --cflags gts)
  LDFLAGS = $(shell $(PKGCONFIG) --libs gts)

  PRESUBDIRS = gts fuzzy libnavi

  $(PLUGINS_all): | presubdirs
endif

LIBS_navigator = core utils vmc netcomm geometry fuzzy gts_obstacle navigator
OBJS_navigator = navigator_plugin.o \
		 navigator_thread.o \
		 joystick_control.o \
		 motor_thread.o \
		 navigator_net_thread.o \
		 navigator.o \
		 pathfinder.o \
		 path_state.o

OBJS_all    = $(OBJS_navigator)

include $(BASEDIR)/etc/buildsys/base.mk

