#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Tools
#                            -------------------
#   Created on Wed Jun 27 14:11:58 2007
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

BASEDIR = ../../..
include $(BASEDIR)/src/firevision/fvconf.mk
include $(LIBSRCDIR)/netcomm/netcomm.mk
include $(BASEDIR)/etc/buildsys/gui.mk

SUBDIRS = firestation

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_fvshmem = $(VISION_CAM_LIBS) utils fvutils
OBJS_fvshmem = shmem.o

LIBS_fvlistfwcams = $(VISION_CAM_LIBS) utils fvcams
OBJS_fvlistfwcams = listfwcams.o

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

LIBS_fvviewer = $(VISION_CAM_LIBS) utils fvcams fvutils fvwidgets fvfilters
OBJS_fvviewer = viewer.o
CFLAGS_viewer    += $(CFLAGS_GTKMM) $(CFLAGS_SDL)
LDFLAGS_fvviewer += $(LDFLAGS_GTKMM) $(LDFLAGS_SDL)

LIBS_fvnet = utils fvutils
OBJS_fvnet = net.o

LIBS_fvshowyuv = fvutils fvwidgets
OBJS_fvshowyuv = show_yuv.o
CFLAGS_show_yuv += $(CFLAGS_SDL)
LDFLAGS_fvshowyuv += $(LDFLAGS_SDL)

OBJS_all = $(OBJS_fvshmem) $(OBJS_fvlistfwcams) $(OBJS_fvstereodecoder) \
	   $(OBJS_fvconverter) $(OBJS_fvtricalcdisp) $(OBJS_fvbb2gettric) \
	   $(OBJS_fvbb2info) $(OBJS_fvbb2rectlut) $(OBJS_fvviewer) \
	   $(OBJS_fvshowyuv) $(OBJS_fvnet)
BINS_all = $(BINDIR)/fvshmem $(BINDIR)/fvconverter $(BINDIR)/fvbb2rectlut \
	   $(BINDIR)/fvnet

ifeq ($(HAVE_TRICLOPS_SDK),1)
  BINS_all += $(BINDIR)/fvtricalcdisp
else
  WARN_TARGETS += warning_tricalcdisp
  WARN_TARGETS += warning_bb2rectlut_triclops
endif

ifeq ($(HAVE_FIREWIRE_CAM),1)
  BINS_all += $(BINDIR)/fvlistfwcams
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
ifeq ($(HAVE_SDL)$(HAVE_GTKMM),11)
  BINS_gui += $(BINDIR)/fvviewer
else
  WARN_TARGETS += warning_viewer
endif
ifeq ($(HAVE_SDL),1)
  BINS_gui += $(BINDIR)/fvshowyuv
else
  WARN_TARGETS += warning_showyuv
endif


ifeq ($(OBJSSUBMAKE),1)
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
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvviewer$(TNORMAL) (SDL[-devel] not installed) $(GUI_ERROR)";
warning_showyuv:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvshowyuv$(TNORMAL) (SDL[-devel] not installed)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

