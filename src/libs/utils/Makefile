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

ifneq ($(PKGCONFIG),)
  HAVE_LIBXMLPP = $(if $(shell $(PKGCONFIG) --exists 'libxml++-2.6'; echo $${?/1/}),1,0)
  HAVE_YAMLCPP   = $(if $(shell $(PKGCONFIG) --exists 'yaml-cpp'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBXMLPP),1)
  CFLAGS_LIBXMLPP=$(shell $(PKGCONFIG) --cflags 'libxml++-2.6')
  LDFLAGS_LIBXMLPP=$(shell $(PKGCONFIG) --libs 'libxml++-2.6')

  CFLAGS  += $(CFLAGS_LIBXMLPP) $(CFLAGS_LIBMAGIC)
  LDFLAGS += $(LDFLAGS_LIBXMLPP) $(LDFLAGS_LIBMAGIC)
else
  FILTER_OUT += graph/rcsoft_graph_map.o
  WARN_TARGETS += warning_libxmlpp
endif
ifeq ($(HAVE_YAMLCPP),1)
  HAVE_YAMLCPP_0_2_6 = $(if $(shell $(PKGCONFIG) --atleast-version='0.2.6' 'yaml-cpp'; echo $${?/1/}),1,0)

  CFLAGS_YAMLCPP  = -DHAVE_YAMLCPP $(shell $(PKGCONFIG) --cflags 'yaml-cpp')
  LDFLAGS_YAMLCPP = $(shell $(PKGCONFIG) --libs 'yaml-cpp')
  ifneq ($(HAVE_YAMLCPP_0_2_6),1)
    CFLAGS_YAMLCPP += -DHAVE_OLD_YAMLCPP
  endif

  CFLAGS  += $(CFLAGS_YAMLCPP)
  LDFLAGS += $(LDFLAGS_YAMLCPP)
else
  FILTER_OUT += graph/yaml_navgraph.o
  WARN_TARGETS += warning_yamlcpp
endif

LIBS_libfawkesutils = stdc++ m rt fawkescore $(LIBS_LIBMAGIC) $(if $(filter-out Darwin,$(OS)),rt) $(if $(filter Linux,$(OS)),dl pthread)
OBJS_libfawkesutils =	$(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
HDRS_libfawkesutils = $(subst $(SRCDIR)/,,$(filter-out $(patsubst %.o,%.h,$(FILTER_OUT)),$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h)))


CFLAGS_fawkesutils_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesutils = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua $(SRCDIR)/*/*/*.tolua)
LDFLAGS_lua_fawkesutils = $(LDFLAGS_LUA)
LIBS_lua_fawkesutils = fawkescore fawkesutils $(TOLUA_LIBS)
OBJS_lua_fawkesutils = fawkesutils_tolua.o

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
.PHONY: warning_libmagic warning_libxmlpp warning_yamlcpp warning_inotify
warning_libmagic:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Restricted file type detection support$(TNORMAL) (file[-devel] (libmagic) not installed)"
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)RCSoft graph utils cannot be built$(TNORMAL) (libxml++[-devel] not installed)"
warning_yamlcpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)YAML navgraph utils cannot be built$(TNORMAL) (yaml-cpp[-devel] not installed)"
warning_inotify:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)File monitoring utils cannot be built$(TNORMAL) (inotify headers not found)"
endif

$(LUALIBDIR)/fawkesutils.$(SOEXT): | $(LIBDIR)/libfawkesutils.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

