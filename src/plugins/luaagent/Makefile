#*****************************************************************************
#             Makefile Build System for Fawkes: LuaAgent Plugin
#                            -------------------
#   Created on Thu Jan 01 14:11:57 2009
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
include $(BUILDSYSDIR)/lua.mk
include $(LIBSRCDIR)/utils/utils.mk
include $(LIBSRCDIR)/tf/tf.mk

CFLAGS  += $(CFLAGS_LUA)

ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
endif

LIBS_luaagent = fawkescore fawkesutils fawkesaspects fawkeslua fawkesblackboard \
		fawkesinterface fawkeslogging SkillerInterface SkillerDebugInterface
OBJS_luaagent = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all      = $(OBJS_luaagent)

ifeq ($(HAVE_LUA),1)
  ifeq ($(HAVE_TOLUA),1)
    PLUGINS_all = $(PLUGINDIR)/luaagent.so
  else
    WARN_TARGETS += warning_tolua
  endif
else
  WARN_TARGETS += warning_lua
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_lua warning_tolua
warning_lua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting LuaAgent Plugin$(TNORMAL) (Lua >= $(LUA_VERSION) not installed)"

warning_tolua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting LuaAgent Plugin$(TNORMAL) (tolua++[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

