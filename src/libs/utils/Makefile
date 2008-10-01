#*****************************************************************************
#              Makefile Build System for Fawkes: Utility Library
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
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

ifneq ($(realpath $(SYSROOT)/usr/include/magic.h),)
  HAVE_LIBMAGIC = 1
else
  ERROR_TARGETS += error_libmagic
endif

LIBS_libfawkesutils = stdc++ fawkescore magic $(if $(filter Linux,$(OS)),dl)
OBJS_libfawkesutils =	$(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))

CFLAGS_fawkesutils_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesutils = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua $(SRCDIR)/*/*/*.tolua)
LDFLAGS_lua_fawkesutils = $(LDFLAGS_LUA)
LIBS_lua_fawkesutils = fawkescore fawkesutils $(TOLUA_LIBS)
OBJS_lua_fawkesutils = fawkesutils_tolua.o

OBJS_all = $(OBJS_libfawkesutils) $(OBJS_lua_fawkesutils)
ifeq ($(HAVE_LIBMAGIC),1)
  LIBS_all  = $(LIBDIR)/libfawkesutils.so
  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LUALIBDIR)/fawkesutils.so
    TARGETS_all += $(SRCDIR)/fawkesutils_tolua.cpp
  else
    WARN_TARGETS += warning_tolua_wrapper
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(ERROR_TARGETS) $(WARN_TARGETS)
.PHONY: error_libmagic
error_libmagic:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Utils cannot be built$(TNORMAL) (file-devel not installed)"
	$(SILENT)exit 1
endif

$(LUALIBDIR)/fawkesutils.so: | $(LIBDIR)/libfawkesutils.so

include $(BASEDIR)/etc/buildsys/base.mk

