#*****************************************************************************
#           Makefile Build System for Fawkes: Configuration Utils
#                            -------------------
#   Created on Sun Jan 07 14:56:02 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/lua.mk

ifneq ($(HAVE_SQLITE),1)
  ERROR_TARGETS += error_sqlite
else
  CFLAGS += $(shell $(PKGCONFIG) --cflags sqlite3)
  LDFLAGS_libconfig = $(shell $(PKGCONFIG) --libs sqlite3)
  LIBS_all  = $(LIBDIR)/libconfig.so
  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LIBDIR)/lua/config.so
    TARGETS_all += $(SRCDIR)/config_tolua.cpp
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(ERROR_TARGETS)
.PHONY: error_sqlite
error_sqlite:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)libconfig cannot be built$(TNORMAL) (SQLite 3 not installed)"
	$(SILENT)exit 1
endif

LIBS_libconfig = stdc++ core utils netcomm
OBJS_libconfig = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

CFLAGS_config_tolua = -Wno-unused-function
TOLUA_config = $(wildcard $(SRCDIR)/*.tolua)
LIBS_lua_config = core config tolua++
OBJS_lua_config = config_tolua.o

OBJS_all = $(OBJS_libconfig) $(OBJS_lua_config)

$(LIBDIR)/lua/config.so: $(LIBDIR)/libconfig.so

include $(BASEDIR)/etc/buildsys/base.mk

