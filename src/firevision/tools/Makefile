#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Tools
#                            -------------------
#   Created on Wed Jun 27 14:11:58 2007
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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#

BASEDIR = ../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS) -Wno-packed -Wno-attributes -Wno-non-virtual-dtor
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_fvshmem = $(VISION_CAM_LIBS) utils fvutils
OBJS_fvshmem = shmem.o

LIBS_listfwcams = $(VISION_CAM_LIBS) utils fvcams
OBJS_listfwcams = listfwcams.o

LIBS_fvconverter = fvutils
OBJS_fvconverter = converter.o

LIBS_fvstereodecoder = $(VISION_CAM_LIBS) utils fvcams
OBJS_fvstereodecoder = stereodecoder.o

LIBS_fvtricalcdisp = $(VISION_CAM_LIBS) utils fvcams fvstereo
OBJS_fvtricalcdisp = tricalcdisp.o

LIBS_fvbb2gettric = $(VISION_CAM_LIBS) utils fvcams fvutils
OBJS_fvbb2gettric = bb2gettric.o

LIBS_fvbb2rectlut = $(VISION_CAM_LIBS) utils fvcams fvutils fvstereo
OBJS_fvbb2rectlut = bb2rectlut.o

LIBS_fvbb2info = $(VISION_CAM_LIBS) utils fvcams fvutils fvstereo
OBJS_fvbb2info = bb2info.o

LIBS_fvviewer = $(VISION_CAM_LIBS) utils fvcams fvutils fvwidgets
OBJS_fvviewer = viewer.o

LIBS_fvshowyuv = fvutils fvwidgets
OBJS_fvshowyuv = show_yuv.o
CFLAGS_show_yuv += $(SDL_CFLAGS)
LDFLAGS_fvshowyuv += $(SDL_LDFLAGS)

OBJS_all = $(OBJS_fvshmem) $(OBJS_listfwcams) $(OBJS_fvstereodecoder) \
	   $(OBJS_fvconverter) $(OBJS_fvtricalcdisp) $(OBJS_fvbb2gettric) \
	   $(OBJS_fvbb2info) $(OBJS_fvbb2rectlut) $(OBJS_fvviewer) \
	   $(OBJS_fvshowyuv)
BINS_all = $(BINDIR)/fvshmem $(BINDIR)/fvconverter $(BINDIR)/fvbb2rectlut \
	   $(BINDIR)/fvshowyuv

ifeq ($(HAVE_TRICLOPS_SDK),1)
  BINS_all += $(BINDIR)/fvtricalcdisp
else
  WARN_TARGETS += warning_tricalcdisp
  WARN_TARGETS += warning_bb2rectlut_triclops
endif

ifeq ($(HAVE_FIREWIRE_CAM),1)
  BINS_all += $(BINDIR)/listfwcams
else
  WARN_TARGETS += warning_listfwcams
endif

ifeq ($(HAVE_BUMBLEBEE2_CAM),1)
  BINS_all += $(BINDIR)/fvbb2gettric $(BINDIR)/fvstereodecoder $(BINDIR)/fvbb2info
else
  WARN_TARGETS += warning_bb2gettric
  WARN_TARGETS += warning_bb2info
  WARN_TARGETS += warning_stereodecoder
  WARN_TARGETS += warning_bb2rectlut_bb2
endif
ifeq ($(HAVE_SDL),1)
  BINS_all += $(BINDIR)/fvviewer
  CFLAGS_viewer += $(SDL_CFLAGS)
  LDFLAGS_fvviewer += $(SDL_LDFLAGS)
else
  WARN_TARGETS += warning_viewer
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_listfwcams:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building listfwcams$(TNORMAL) (no Firewire camera support)";
warning_stereodecoder:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building stereodecoder$(TNORMAL) (no Bumblebee2 camera support)";
warning_tricalcdisp:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvtricalcdisp$(TNORMAL) (no Triclops SDK support)";
warning_bb2gettric:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvbb2gettric$(TNORMAL) (no Bumblebee2 camera support)";
warning_bb2rectlut_nobb2:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Building fvbb2rectlut w/o retrieval and verification support$(TNORMAL) (no BB2 support)";
warning_bb2rectlut_triclops:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Building fvbb2rectlut w/o retrieval and limited verification support$(TNORMAL) (no Triclops SDK)";
warning_bb2info:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvbb2info$(TNORMAL) (no Bumblebee2 camera support)";
warning_viewer:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvviewer$(TNORMAL) (SDL[-devel] not installed)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

