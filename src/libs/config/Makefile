#*****************************************************************************
#           Makefile Build System for Fawkes: Configuration Utils
#                            -------------------
#   Created on Sun Jan 07 14:56:02 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

ifneq ($(HAVE_SQLITE),1)
  ERROR_TARGETS += error_sqlite
else
  CFLAGS += $(shell $(PKGCONFIG) --cflags sqlite3)
  LDFLAGS_libfawkesconfig = $(shell $(PKGCONFIG) --libs sqlite3)
  LIBS_all  = $(LIBDIR)/libfawkesconfig.so
  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LUALIBDIR)/fawkesconfig.so
    TARGETS_all += $(SRCDIR)/fawkesconfig_tolua.cpp
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(ERROR_TARGETS)
.PHONY: error_sqlite
error_sqlite:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)libconfig cannot be built$(TNORMAL) (SQLite 3 not installed)"
	$(SILENT)exit 1
endif

LIBS_libfawkesconfig = stdc++ fawkescore fawkesutils fawkesnetcomm fawkeslogging
OBJS_libfawkesconfig =$(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))
HDRS_libfawkesconfig = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

CFLAGS_fawkesconfig_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesconfig = $(wildcard $(SRCDIR)/*.tolua)
LDFLAGS_lua_fawkesconfig = $(LDFLAGS_LUA)
LIBS_lua_fawkesconfig = fawkescore fawkesconfig $(TOLUA_LIBS)
OBJS_lua_fawkesconfig = fawkesconfig_tolua.o

OBJS_all = $(OBJS_libfawkesconfig) $(OBJS_lua_fawkesconfig)

$(LUALIBDIR)/fawkesconfig.so: | $(LIBDIR)/libfawkesconfig.so

include $(BUILDSYSDIR)/base.mk

