#*****************************************************************************
#            Makefile Build System for Fawkes: Player Plugin
#                            -------------------
#   Created on Wed Aug 06 14:24:52 2008
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
include $(BUILDSYSDIR)/download.mk

ifeq ($(CC),gcc)
  CFLAGS += -Wno-cpp
endif
ifeq ($(CC),clang)
  CFLAGS += "-Wno-\#warnings"
endif


ifeq ($(HAVE_PLAYERCC),1)
  LDFLAGS += $(LDFLAGS_PLAYERCC)
  CFLAGS  += $(CFLAGS_PLAYERCC)
  LIBS_player = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		fawkesinterface fawkesnetcomm \
		Laser360Interface MotorInterface ObjectPositionInterface
  OBJS_player = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
  PLUGINS_all = $(PLUGINDIR)/player.so
else
  WARN_TARGETS += warning_playercc
endif

OBJS_all = $(OBJS_player)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_playercc
warning_playercc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting player plugin$(TNORMAL) (libplayerc++ not installed)"
endif

include $(BUILDSYSDIR)/base.mk

