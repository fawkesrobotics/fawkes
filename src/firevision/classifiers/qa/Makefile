#*****************************************************************************
#         Makefile Build System for Fawkes : FireVision Classifiers QA
#                            -------------------
#   Created on Mon Dec 10 23:30:29 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

BASEDIR = ../../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

OBJS_fv_qa_facesclassifier := qa_facesclassifier.o
LIBS_fv_qa_facesclassifier := fvclassifiers fvutils fvwidgets fvfilters
CFLAGS_qa_facesclassifier += $(CFLAGS_SDL)
LDFLAGS_fv_qa_facesclassifier += $(LDFLAGS_SDL)

OBJS_all = $(OBJS_fv_qa_facesclassifier)
BINS_all = $(BINDIR)/fv_qa_facesclassifier

include $(BASEDIR)/etc/buildsys/base.mk

