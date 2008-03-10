#*****************************************************************************
#       Makefile Build System for Fawkes: Skill Execution Runtime Plugin
#                            -------------------
#   Created on Fri Feb 08 00:47:23 2008
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

CFLAGS  += -DSKILLDIR=\"$(SKILLDIR)\" $(CFLAGS_LUA)
LDFLAGS += $(LDFLAGS_LUA)

LIBS_skiller = core utils netcomm
OBJS_skiller = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_skiller)

ifeq ($(HAVE_LUA),1)
  PLUGINS_all = $(PLUGINDIR)/skiller.so
else
  WARN_TARGETS += warning_lua
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_lua
warning_lua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Skiller Plugin$(TNORMAL) (install Lua >= $(LUA_MINVERSION))"
endif

include $(BASEDIR)/etc/buildsys/base.mk

