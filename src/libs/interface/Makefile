#*****************************************************************************
#               Makefile Build System for Fawkes : Interface lib
#                            -------------------
#   Created on Fri Jan 12 18:08:27 2007
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

LIBS_libfawkesinterface = fawkescore fawkesutils
OBJS_libfawkesinterface = interface.o interface_info.o message.o message_queue.o field_iterator.o
HDRS_libfawkesinterface = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

CFLAGS_fawkesinterface_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesinterface = $(wildcard $(SRCDIR)/*.tolua)
LDFLAGS_lua_fawkesinterface = $(LDFLAGS_LUA)
LIBS_lua_fawkesinterface = fawkescore fawkesinterface $(TOLUA_LIBS)
OBJS_lua_fawkesinterface = fawkesinterface_tolua.o

OBJS_all = $(OBJS_libfawkesinterface) $(OBJS_lua_fawkesinterface) $(SRCDIR)/interface_tolua.cpp
LIBS_all = $(LIBDIR)/libfawkesinterface.so
ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkesinterface.so
  TARGETS_all += $(SRCDIR)/fawkesinterface_tolua.cpp
else
  all: warning_tolua_wrapper
endif

$(LUALIBDIR)/fawkesinterface.so: | $(LIBDIR)/libfawkesinterface.so

include $(BUILDSYSDIR)/base.mk

