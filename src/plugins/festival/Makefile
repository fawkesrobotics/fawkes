#*****************************************************************************
#            Makefile Build System for Fawkes: Festival Plugin
#                            -------------------
#   Created on Tue Oct 28 14:50:37 2008
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

ifeq ($(HAVE_FESTIVAL),1)
  LDFLAGS += $(LDFLAGS_FESTIVAL)
  CFLAGS  += $(CFLAGS_FESTIVAL)
  LIBS_festival = fawkescore fawkesutils fawkesaspects fawkesblackboard fawkesinterface \
		  SpeechSynthInterface $(LIBS_FESTIVAL)
  OBJS_festival = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
  PLUGINS_all = $(PLUGINDIR)/festival.so
else
  WARN_TARGETS += warning_festival
endif

OBJS_all = $(OBJS_festival)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_festival
warning_festival:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting festival plugin$(TNORMAL) ($(FESTIVAL_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk

