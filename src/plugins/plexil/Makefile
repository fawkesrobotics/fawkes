#*****************************************************************************
#           Makefile Build System for Fawkes: PLEXIL Executive Plugin
#                            -------------------
#   Created on Mon Aug 13 11:21:49 2018
#   Copyright (C) 2006-2018 by Tim Niemueller
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
include $(BUILDSYSDIR)/clips.mk

LIBS_plexil = \
	fawkescore fawkesutils fawkesaspects fawkesblackboard \
	fawkesinterface \
  PlexilAppFramework PlanDebugListener LuvListener PlexilSockets
OBJS_plexil = plexil_plugin.o plexil_thread.o clock_adapter.o timer_thread.o \
  log_adapter.o

OBJS_all = $(OBJS_plexil)

ifeq ($(HAVE_PLEXIL),1)
  CFLAGS  += $(CFLAGS_PLEXIL)
  LDFLAGS += $(LDFLAGS_PLEXIL)

  PLUGINS_all = $(PLUGINDIR)/plexil.so
else
  WARN_TARGETS += warning_plexil
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_plexil
warning_plexil:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting plexil plugin$(TNORMAL) ($(PLEXIL_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
