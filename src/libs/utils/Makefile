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
else
  ERROR_TARGETS += error_libmagic
endif

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLPP = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBXMLPP),1)
  CFLAGS_LIBXMLPP=$(shell $(PKGCONFIG) --cflags 'libxml++-2.6')
  LDFLAGS_LIBXMLPP=$(shell $(PKGCONFIG) --libs 'libxml++-2.6')

  CFLAGS  += $(CFLAGS_LIBXMLPP)
  LDFLAGS += $(LDFLAGS_LIBXMLPP)
else
  FILTER_OUT += graph/%.o
  WARN_TARGETS += warning_libxmlpp
endif

LIBS_libfawkesutils = stdc++ m fawkescore magic $(if $(filter-out Darwin,$(OS)),rt) $(if $(filter Linux,$(OS)),dl)
OBJS_libfawkesutils =	$(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
HDRS_libfawkesutils = $(subst $(SRCDIR)/,,$(filter-out $(patsubst %.o,%.h,$(FILTER_OUT)),$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h)))


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
.PHONY: error_libmagic warning_libxmlpp
error_libmagic:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Utils cannot be built$(TNORMAL) (file-devel not installed)"
	$(SILENT)exit 1
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Graph utils cannot be built$(TNORMAL) (libxml++[-devel] not installed)"
endif

$(LUALIBDIR)/fawkesutils.so: | $(LIBDIR)/libfawkesutils.so

include $(BUILDSYSDIR)/base.mk

