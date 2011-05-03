#*****************************************************************************
#        Makefile Build System for Fawkes : Point Cloud Viewer
#                            -------------------
#   Created on Fri Apr 01 14:39:58 2011
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

LIBS_ffpclviewer = stdc++ m fawkescore fawkesutils fawkesnetcomm \
		 fawkesblackboard fawkesinterface fvutils fvcams \
		 fawkesopenni_client_utils \
		 ObjectPositionInterface HumanSkeletonInterface
OBJS_ffpclviewer = pclviewer.o trackball.o skel_drawer.o \
		   transfer_thread.o

OBJS_all = $(OBJS_ffpclviewer)

CFLAGS  += $(CFLAGS_GL) $(CFLAGS_GLU) $(CFLAGS_GLUT)
LDFLAGS += $(LDFLAGS_GL) $(LDFLAGS_GLU) $(LDFLAGS_GLUT)

ifeq ($(HAVE_GL)$(HAVE_GLU)$(HAVE_GLUT),111)
  BINS_gui = $(BINDIR)/ffpclviewer
else
  WARN_TARGETS += warning_opengl
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_opengl
warning_opengl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL viewer$(TNORMAL) (OpenGL not available, mesa-libGL[-devel], libmesa-libGLU[-devel] or freeglut[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
