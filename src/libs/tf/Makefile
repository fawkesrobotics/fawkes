#*****************************************************************************
#          Makefile for Fawkes Transforms Library (based on ROS tf)
#                            -------------------
#   Created on Tue Oct 18 16:14:13 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libfawkestf = fawkescore fawkesutils fawkesblackboard fawkesinterface \
	                 TransformInterface
OBJS_libfawkestf = buffer_core.o time_cache.o static_cache.o exceptions.o \
	                 transformer.o transform_listener.o transform_publisher.o
HDRS_libfawkestf = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h  $(SRCDIR)/*/*/*.h ))

CFLAGS_fawkestf_tolua = -Wno-unused-function $(CFLAGS_LUA) $(CFLAGS)
TOLUA_fawkestf = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua)
LDFLAGS_lua_fawkestf = $(LDFLAGS_LUA)
LIBS_lua_fawkestf = fawkescore fawkesutils fawkestf $(TOLUA_LIBS)
OBJS_lua_fawkestf = fawkestf_tolua.o

OBJS_all = $(OBJS_libfawkestf) $(OBJS_lua_fawkestf)

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF) -O2
  LDFLAGS += $(LDFLAGS_TF)
  LIBS_all = $(LIBDIR)/libfawkestf.so

  ifeq ($(HAVE_TOLUA),1)
    LIBS_all += $(LUALIBDIR)/fawkestf.so
    TARGETS_all += $(SRCDIR)/fawkestf_tolua.cpp
  else
    WARN_TARGETS += warning_tolua_wrapper
  endif
else
	ifneq ($(HAVE_BULLET),1)
    WARN_TARGETS += warning_bullet
  endif
	ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

$(LUALIBDIR)/fawkestf.$(SOEXT): | $(LIBDIR)/libfawkestf.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_bullet warning_cpp11
warning_bullet:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms library$(TNORMAL) (bullet[-devel] not installed)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting transforms library$(TNORMAL) (C++11 required)"
endif

include $(BUILDSYSDIR)/base.mk
