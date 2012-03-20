#*****************************************************************************
#        Makefile Build System for Fawkes : Skeleton Visualization
#                            -------------------
#   Created on Wed Mar 02 11:12:39 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/plugins/openni/openni.mk

LIBS_ffskelgui = stdc++ m fawkescore fawkesutils fawkesnetcomm \
		 fawkesblackboard fawkesinterface fvutils fvcams \
		 fawkesopenni_client_utils \
		 HumanSkeletonInterface HumanSkeletonProjectionInterface \
		 ObjectPositionInterface
OBJS_ffskelgui = main.o skel_drawer.o texture_drawer.o \
		 image_drawer.o depth_drawer.o

OBJS_all = $(OBJS_ffskelgui)

CFLAGS  += $(CFLAGS_GL) $(CFLAGS_GLUT)
LDFLAGS += $(LDFLAGS_GL) $(LDFLAGS_GLUT)

ifeq ($(HAVE_GL)$(HAVE_GLUT),11)
  BINS_gui = $(BINDIR)/ffskelgui
else
  WARN_TARGETS += warning_opengl
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_opengl
warning_opengl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting skeleton GUI$(TNORMAL) (OpenGL not available, mesa-libGL[-devel] or freeglut[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
