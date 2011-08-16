#*****************************************************************************
#         Makefile Build System for Fawkes: Fawkes Lua Integration
#                            -------------------
#   Created on Fri May 23 11:21:28 2008
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

LIBS_libfawkeslua = fawkescore fawkesutils fawkesconfig fawkesblackboard \
		    fawkesinterface fawkeslogging $(TOLUA_LIBS)
OBJS_libfawkeslua = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libfawkeslua = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

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


include $(BUILDSYSDIR)/base.mk

