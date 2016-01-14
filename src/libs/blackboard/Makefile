#*****************************************************************************
#               Makefile Build System for Fawkes : BlackBoard
#                            -------------------
#   Created on Wed Oct 04 10:49:27 2006
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

LIBS_libfawkesblackboard = fawkescore fawkesutils fawkesinterface fawkesnetcomm fawkeslogging
OBJS_libfawkesblackboard = $(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))))
HDRS_libfawkesblackboard = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h))

CFLAGS_fawkesblackboard_tolua = -Wno-unused-function $(CFLAGS_LUA) $(CFLAGS_CPP11)
TOLUA_fawkesblackboard = blackboard.tolua
LDFLAGS_lua_fawkesblackboard = $(LDFLAGS_LUA)
LIBS_lua_fawkesblackboard = fawkescore fawkesutils fawkesblackboard $(TOLUA_LIBS)
OBJS_lua_fawkesblackboard = fawkesblackboard_tolua.o

OBJS_all =	$(OBJS_libfawkesblackboard) $(OBJS_lua_fawkesblackboard)
LIBS_all =	$(LIBDIR)/libfawkesblackboard.$(SOEXT)

ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkesblackboard.$(SOEXT)
  TARGETS_all += $(SRCDIR)/fawkesblackboard_tolua.cpp
else
  WARN_TARGETS += warning_tolua_wrapper
endif

$(LUALIBDIR)/fawkesblackboard.$(SOEXT): | $(LIBDIR)/libfawkesblackboard.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

