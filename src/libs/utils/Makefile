#*****************************************************************************
#              Makefile Build System for Fawkes: Utility Library
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
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

FILTER_OUT=%_tolua.o

ifneq ($(wildcard $(SYSROOT)/usr/include/magic.h $(SYSROOT)/opt/local/include/magic.h),)
  HAVE_LIBMAGIC = 1
  LIBS_LIBMAGIC = magic
  CFLAGS_LIBMAGIC = -DHAVE_LIBMAGIC
else
  WARN_TARGETS += warning_libmagic
endif

LIBS_libfawkesutils = stdc++ m rt fawkescore $(LIBS_LIBMAGIC) $(if $(filter-out Darwin,$(OS)),rt) $(if $(filter Linux,$(OS)),dl pthread)
OBJS_libfawkesutils =	$(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
HDRS_libfawkesutils = $(subst $(SRCDIR)/,,$(filter-out $(patsubst %.o,%.h,$(FILTER_OUT)),$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h)))


CFLAGS_fawkesutils_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesutils = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua $(SRCDIR)/*/*/*.tolua)
LDFLAGS_lua_fawkesutils = $(LDFLAGS_LUA)
LIBS_lua_fawkesutils = fawkescore fawkesutils $(TOLUA_LIBS)
OBJS_lua_fawkesutils = fawkesutils_tolua.o

CFLAGS  += $(CFLAGS_LIBMAGIC)
LDFLAGS += $(LDFLAGS_LIBMAGIC)

OBJS_all = $(OBJS_libfawkesutils) $(OBJS_lua_fawkesutils)
LIBS_all  = $(LIBDIR)/libfawkesutils.so
ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkesutils.so
  TARGETS_all += $(SRCDIR)/fawkesutils_tolua.cpp
else
  WARN_TARGETS += warning_tolua_wrapper
endif

ifneq ($(HAVE_INOTIFY),1)
  WARN_TARGETS += warning_inotify
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_libmagic warning_inotify
warning_libmagic:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Restricted file type detection support$(TNORMAL) (file[-devel] (libmagic) not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)File monitoring utils cannot be built$(TNORMAL) (inotify headers not found)"
endif

$(LUALIBDIR)/fawkesutils.$(SOEXT): | $(LIBDIR)/libfawkesutils.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

