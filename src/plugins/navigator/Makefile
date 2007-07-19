#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator Plugin
#                            -------------------
#   Created on Tue May 29 18:41:17 2007
#   copyright (C) 2007 by Martin Liebenberg, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

BASEDIR = ../../..

CFLAGS = -g `gts-config --cflags`

ifneq ($(realpath /usr/include/vmc/LayerClasses/VMC_API.h),)
  HAVE_VMC = 1
endif

ifneq ($(HAVE_VMC),1)
  WARN_TARGETS += warning_vmc
else
  PLUGINS_all = $(PLUGINDIR)/navigator.so
endif

ifneq ($(SRCDIR),)
all: $(WARN_TARGETS)

.PHONY: warning_vmc
warning_vmc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Navigator Plugin$(TNORMAL) (install libvmc)"
endif

PRESUBDIRS = gts fuzzy libnavi

LIBS_navigator = core utils vmc netcomm fuzzy gts_obstacle navigator
OBJS_navigator = navigator_plugin.o \
		 navigator_thread.o \
		 joystick_control.o \
		 motor_thread.o \
		 navigator_net_thread.o \
		 navigator.o \
		 pathfinder.o \
		 path_state.o

		 
LDFLAGS_navigator = `gts-config --libs`

		 

OBJS_all    = $(OBJS_navigator)

include $(BASEDIR)/etc/buildsys/base.mk
