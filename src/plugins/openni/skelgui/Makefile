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

LIBS_ffskelgui = stdc++ m glut GL fawkescore fawkesutils fawkesnetcomm \
		 fawkesblackboard fawkesinterface fvutils fvcams \
		 HumanSkeletonInterface HumanSkeletonProjectionInterface
OBJS_ffskelgui = main.o colors.o skel_drawer.o texture_drawer.o \
		 image_drawer.o depth_drawer.o

OBJS_all =	$(OBJS_ffskelgui)

BINS_gui =	$(BINDIR)/ffskelgui

include $(BUILDSYSDIR)/base.mk

