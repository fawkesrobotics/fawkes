#*****************************************************************************
#              Makefile Build System for Fawkes: Geometry Library
#                            -------------------
#   Created on Sun Oct 14 18:48:54 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/lua.mk

HAVE_OPENCV = $(if $(shell $(PKGCONFIG) --exists 'opencv'; echo $${?/1/}),1,0)
ifeq ($(HAVE_OPENCV),1)
  CFLAGS += -DHAVE_OPENCV $(shell $(PKGCONFIG) --cflags 'opencv')
  LDFLAGS += $(subst -lhighgui,,$(shell $(PKGCONFIG) --libs 'opencv'))
endif

SUBDIRS = gtk

LIBS_libfawkesgeometry = stdc++
OBJS_libfawkesgeometry = matrix.o		\
			 vector.o		\
			 printable.o		\
			 transformable.o	\
			 hom_transform.o	\
			 hom_coord.o		\
			 hom_point.o		\
			 hom_vector.o		\
			 hom_polar.o		\
			 hom_pose.o		\
			 hom_pose_2d.o		\
			 bezier.o		\
			 spline.o		\
			 line_segment.o		\
			 hom_pose_2d.o

CFLAGS_fawkesgeometry_tolua = -Wno-unused-function $(CFLAGS_LUA)
TOLUA_fawkesgeometry = $(wildcard $(SRCDIR)/*.tolua $(SRCDIR)/*/*.tolua $(SRCDIR)/*/*/*.tolua)
LDFLAGS_lua_fawkesgeometry = $(LDFLAGS_LUA)
LIBS_lua_fawkesgeometry = fawkescore fawkesgeometry $(TOLUA_LIBS)
OBJS_lua_fawkesgeometry = fawkesgeometry_tolua.o

OBJS_all = $(OBJS_libfawkesgeometry) $(OBJS_lua_fawkesgeometry)
LIBS_all = $(LIBDIR)/libfawkesgeometry.so
ifeq ($(HAVE_TOLUA),1)
  LIBS_all += $(LUALIBDIR)/fawkesgeometry.so
  TARGETS_all += $(SRCDIR)/fawkesgeometry_tolua.cpp
else
  WARN_TARGETS += warning_tolua_wrapper
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
endif

$(LUALIBDIR)/fawkesutils.so: | $(LIBDIR)/libfawkesutils.so

include $(BASEDIR)/etc/buildsys/base.mk

