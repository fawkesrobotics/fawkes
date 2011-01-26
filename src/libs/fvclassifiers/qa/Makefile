#*****************************************************************************
#         Makefile Build System for Fawkes : FireVision Classifiers QA
#                            -------------------
#   Created on Mon Dec 10 23:30:29 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

OBJS_fv_qa_facesclassifier := qa_facesclassifier.o
LIBS_fv_qa_facesclassifier := fvclassifiers fvutils fvwidgets fvfilters fvcams
CFLAGS_qa_facesclassifier += $(CFLAGS_SDL)
LDFLAGS_fv_qa_facesclassifier += $(LDFLAGS_SDL)

OBJS_fv_qa_siftclassifier := qa_siftclassifier.o
LIBS_fv_qa_siftclassifier := fvclassifiers fvutils fvwidgets fvfilters $(LIBS_SIFT)
CFLAGS_qa_siftclassifier += $(CFLAGS_SIFT)

OBJS_fv_qa_surfclassifier := qa_surfclassifier.o
LIBS_fv_qa_surfclassifier := fvclassifiers fvutils fvwidgets fvfilters $(LIBS_SURF)
CFLAGS_qa_surfclassifier += $(CFLAGS_SURF)

OBJS_fv_qa_siftppclassifier := qa_siftppclassifier.o
LIBS_fv_qa_siftppclassifier := fvclassifiers fvutils fvwidgets fvfilters $(LIBS_SIFTPP)
CFLAGS_qa_siftppclassifier += $(CFLAGS_SIFTPP)
#LDFLAGS_fv_qa_siftppclassifier += $(LDFLAGS_SIFTPP)

OBJS_all = $(OBJS_fv_qa_facesclassifier) $(OBJS_fv_qa_siftclassifier) $(OBJS_fv_qa_surfclassifier) $(OBJS_fv_qa_siftppclassifier)

ifeq ($(HAVE_OPENCV),1)
  BINS_all = $(BINDIR)/fv_qa_facesclassifier
endif

ifeq ($(HAVE_SIFT),1)
  BINS_all += $(BINDIR)/fv_qa_siftclassifier
endif

ifeq ($(HAVE_SURF),1)
  BINS_all += $(BINDIR)/fv_qa_surfclassifier
endif

ifeq ($(HAVE_SIFTPP),1)
  BINS_all += $(BINDIR)/fv_qa_siftppclassifier
endif

include $(BUILDSYSDIR)/base.mk

