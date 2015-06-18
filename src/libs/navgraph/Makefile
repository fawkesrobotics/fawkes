#*****************************************************************************
#              Makefile Build System for Fawkes: NavGraph Library
#                            -------------------
#   Created on Sun Sep 03 14:14:14 2006
#   Copyright (C) 2006-2015 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/eigen3.mk

FILTER_OUT=%_tolua.o

LIBS_libfawkesnavgraph = stdc++ m fawkescore fawkesutils
OBJS_libfawkesnavgraph = navgraph.o navgraph_node.o navgraph_edge.o navgraph_path.o \
			 yaml_navgraph.o search_state.o \
                         $(subst $(SRCDIR)/,,$(patsubst %.cpp,%.o,$(wildcard $(SRCDIR)/constraints/*.cpp)))
HDRS_libfawkesnavgraph = $(OBJS_libfawkesnavgraph:%.o=%.h)

CFLAGS_fawkesnavgraph_tolua = -Wno-unused-function $(CFLAGS_LUA) $(CFLAGS_CPP11)
TOLUA_fawkesnavgraph = navgraph.tolua navgraph_node.tolua
LDFLAGS_lua_fawkesnavgraph = $(LDFLAGS_LUA)
LIBS_lua_fawkesnavgraph = fawkescore fawkesutils fawkesnavgraph $(TOLUA_LIBS)
OBJS_lua_fawkesnavgraph = fawkesnavgraph_tolua.o

OBJS_all = $(OBJS_libfawkesnavgraph) $(OBJS_lua_fawkesnavgraph)

ifeq ($(HAVE_NAVGRAPH),1)
  CFLAGS  += $(CFLAGS_NAVGRAPH)  $(CFLAGS_EIGEN3)  $(CFLAGS_YAMLCPP)
  LDFLAGS += $(LDFLAGS_NAVGRAPH) $(LDFLAGS_EIGEN3) $(LDFLAGS_YAMLCPP)

  LIBS_all  += $(LIBDIR)/libfawkesnavgraph.$(SOEXT)

  SUBDIRS = aspect generators

  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LUALIBDIR)/fawkesnavgraph.$(SOEXT)
    TARGETS_all += $(SRCDIR)/fawkesnavgraph_tolua.cpp
  else
    WARN_TARGETS += warning_tolua_wrapper
  endif
else
  WARN_TARGETS += warning_navgraph
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_navgraph
warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)navgraph library cannot be built$(TNORMAL) ($(NAVGRAPH_ERROR))"
endif

$(LUALIBDIR)/fawkesnavgraph.$(SOEXT): | $(LIBDIR)/libfawkesnavgraph.$(SOEXT)

ifneq ($(MAKECMDGOALS),clean)
aspect: | $(LIBDIR)/libfawkesnavgraph.$(SOEXT)
generators: | $(LIBDIR)/libfawkesnavgraph.$(SOEXT)
endif

include $(BUILDSYSDIR)/base.mk

