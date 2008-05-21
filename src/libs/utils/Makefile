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

ifneq ($(realpath /usr/include/magic.h),)
  HAVE_LIBMAGIC = 1
else
  ERROR_TARGETS += error_libmagic
endif

LIBS_libutils = stdc++ core magic $(if $(filter Linux,$(OS)),dl) bfd
OBJS_libutils =	$(filter-out %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))

CFLAGS_utils_tolua = -Wno-unused-function
TOLUA_utils = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua $(SRCDIR)/*/*/*.tolua)
LIBS_lua_utils = core utils tolua++
OBJS_lua_utils = utils_tolua.o

OBJS_all = $(OBJS_libutils) $(OBJS_lua_utils)
ifeq ($(HAVE_LIBMAGIC),1)
  LIBS_all  = $(LIBDIR)/libutils.so
  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LIBDIR)/lua/utils.so
    TARGETS_all += $(SRCDIR)/utils_tolua.cpp
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

$(LIBDIR)/lua/utils.so: | $(LIBDIR)/libutils.so

include $(BASEDIR)/etc/buildsys/base.mk

