#*****************************************************************************
#              Makefile Build System for Fawkes: Geometry Library
#                            -------------------
#   Created on Sun Oct 14 18:48:54 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/gui.mk

LIBS_libfawkesgeometrygtk = stdc++ fawkesgeometry
OBJS_libfawkesgeometrygtk = geom_drawing_area.o		\
			    geom_drawer.o		\
			    hom_point_drawer.o		\
			    hom_vector_drawer.o		\
			    line_segment_drawer.o	\
			    bezier_drawer.o		\
			    spline_drawer.o		\
			    drawing_manipulator.o
HDRS_libfawkesgeometrygtk = $(patsubst %.o,%.h,$(OBJS_libfawkesgeometrygtk))
INST_HDRS_SUBDIR_libfawkesgeometrygtk = geometry/gtk

ifeq ($(HAVE_GUI),1)
  CFLAGS  += $(CFLAGS_GUI)
  LDFLAGS += $(LDFLAGS_GUI)

  LIBS_gui = $(LIBDIR)/libfawkesgeometrygtk.so
else
  WARN_TARGETS += warning_geometrygtk
endif

OBJS_all = $(OBJS_libfawkesgeometrygtk)

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: warning_geometrygtk
warning_geometrygtk:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gtk drawers of the geometry library$(TNORMAL)"
endif

include $(BASEDIR)/etc/buildsys/base.mk

