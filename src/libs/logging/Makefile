#*****************************************************************************
#           Makefile Build System for Fawkes: Logging Utils
#                            -------------------
#   Created on Sat May 28 00:16:57 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_all  = $(LIBDIR)/libfawkeslogging.so

ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkeslogging.so
  TARGETS_all += $(SRCDIR)/fawkeslogging_tolua.cpp
endif

LIBS_libfawkeslogging = stdc++ fawkescore fawkesnetcomm
OBJS_libfawkeslogging =$(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))
HDRS_libfawkeslogging = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

CFLAGS_fawkeslogging_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkeslogging = $(wildcard $(SRCDIR)/*.tolua)
LDFLAGS_lua_fawkeslogging= $(LDFLAGS_LUA)
LIBS_lua_fawkeslogging = fawkescore fawkeslogging $(TOLUA_LIBS)
OBJS_lua_fawkeslogging = fawkeslogging_tolua.o

OBJS_all = $(OBJS_libfawkeslogging) $(OBJS_lua_fawkeslogging)

$(LUALIBDIR)/fawkeslogging.so: | $(LIBDIR)/libfawkeslogging.so

include $(BUILDSYSDIR)/base.mk

