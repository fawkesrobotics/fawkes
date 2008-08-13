#*****************************************************************************
#         Makefile Build System for Fawkes: Fawkes Lua Integration
#                            -------------------
#   Created on Fri May 23 11:21:28 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/lua.mk

LIBS_libfawkeslua = fawkescore fawkesutils $(TOLUA_LIBS)
OBJS_libfawkeslua = $(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))

CFLAGS += $(CFLAGS_LUA)
LDFLAGS_libfawkeslua += $(LDFLAGS_LUA)

OBJS_all = $(OBJS_libfawkeslua)

ifeq ($(HAVE_LUA),1)
  ifeq ($(HAVE_TOLUA),1)
    LIBS_all = $(LIBDIR)/libfawkeslua.so
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
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Fawkes Lua integration$(TNORMAL) (Lua >= $(LUA_MINVERSION) not installed)"

warning_tolua:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Fawkes Lua integration$(TNORMAL) (tolua++[-devel] not installed)"
endif


include $(BASEDIR)/etc/buildsys/base.mk

