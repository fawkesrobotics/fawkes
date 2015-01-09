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

SUBDIRS=aspect

ifneq ($(PKGCONFIG),)
  HAVE_YAMLCPP   = $(if $(shell $(PKGCONFIG) --exists 'yaml-cpp'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_YAMLCPP),1)
  HAVE_YAMLCPP_0_2_6 = $(if $(shell $(PKGCONFIG) --atleast-version='0.2.6' 'yaml-cpp'; echo $${?/1/}),1,0)

  CFLAGS_YAMLCPP  = -DHAVE_YAMLCPP $(shell $(PKGCONFIG) --cflags 'yaml-cpp')
  LDFLAGS_YAMLCPP = $(shell $(PKGCONFIG) --libs 'yaml-cpp')
  ifneq ($(HAVE_YAMLCPP_0_2_6),1)
    CFLAGS_YAMLCPP += -DHAVE_OLD_YAMLCPP
  endif

  HAVE_YAMLCPP_0_5 = $(if $(shell $(PKGCONFIG) --atleast-version='0.5' 'yaml-cpp'; echo $${?/1/}),1,0)
  ifeq ($(HAVE_YAMLCPP_0_5),1)
    CFLAGS_YAMLCPP += -DHAVE_YAMLCPP_0_5
  endif

  CFLAGS  += $(CFLAGS_YAMLCPP)
  LDFLAGS += $(LDFLAGS_YAMLCPP)
endif

LIBS_libfawkesnavgraph = stdc++ m fawkescore fawkesutils
OBJS_libfawkesnavgraph = navgraph.o navgraph_node.o navgraph_edge.o yaml_navgraph.o \
                         $(subst $(SRCDIR)/,,$(patsubst %.cpp,%.o,$(wildcard $(SRCDIR)/constraints/*.cpp)))
HDRS_libfawkesnavgraph = $(OBJS_libfawkesnavgraph:%.o=%.h)

CFLAGS_fawkesnavgraph_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesnavgraph = navgraph.tolua navgraph_node.tolua
LDFLAGS_lua_fawkesnavgraph = $(LDFLAGS_LUA)
LIBS_lua_fawkesnavgraph = fawkescore fawkesutils fawkesnavgraph $(TOLUA_LIBS)
OBJS_lua_fawkesnavgraph = fawkesnavgraph_tolua.o

OBJS_all = $(OBJS_libfawkesnavgraph) $(OBJS_lua_fawkesnavgraph)

ifeq ($(HAVE_YAMLCPP)$(HAVE_CPP11),11)
  CFLAGS += $(CFLAGS_CPP11)

  LIBS_all  = $(LIBDIR)/libfawkesnavgraph.$(SOEXT)

  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LUALIBDIR)/fawkesnavgraph.$(SOEXT)
    TARGETS_all += $(SRCDIR)/fawkesnavgraph_tolua.cpp
  else
    WARN_TARGETS += warning_tolua_wrapper
  endif
else
  ifneq ($(HAVE_YAMLCPP),1)
    WARN_TARGETS += warning_yamlcpp
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_yamlcpp warning_cpp11
warning_yamlcpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph library cannot be built$(TNORMAL) (yaml-cpp[-devel] not installed)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph library cannot be built$(TNORMAL) (compiler does not support C++11)"
endif

$(LUALIBDIR)/fawkesnavgraph.$(SOEXT): | $(LIBDIR)/libfawkesnavgraph.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

