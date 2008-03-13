#*****************************************************************************
#               Makefile Build System for Fawkes : Interface lib
#                            -------------------
#   Created on Fri Jan 12 18:08:27 2007
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

LIBS_libinterface = core
OBJS_libinterface = interface.o interface_info.o message.o message_queue.o

CFLAGS_interface_tolua = -Wno-unused-function
TOLUA_interface = $(wildcard $(SRCDIR)/*.tolua)
LIBS_lua_interface = core interface tolua++
OBJS_lua_interface = interface_tolua.o

OBJS_all = $(OBJS_libinterface) $(OBJS_lua_interface) $(SRCDIR)/interface_tolua.cpp
LIBS_all = $(LIBDIR)/libinterface.so
ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LIBDIR)/lua/interface.so
  TARGETS_all += $(SRCDIR)/interface_tolua.cpp
else
  all: warning_tolua_wrapper
endif

$(LIBDIR)/lua/interface.so: $(LIBDIR)/libinterface.so

include $(BASEDIR)/etc/buildsys/base.mk

