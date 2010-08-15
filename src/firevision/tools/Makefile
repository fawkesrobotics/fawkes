#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Tools
#                            -------------------
#   Created on Wed Jun 27 14:11:58 2007
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(FVSRCDIR)/fvconf.mk
include $(BUILDSYSDIR)/gui.mk
include $(BUILDCONFDIR)/netcomm/netcomm.mk

CFLAGS   += $(VISION_CFLAGS) $(CFLAGS_AVAHI)
LDFLAGS  += $(VISION_LDFLAGS) $(LDFLAGS_AVAHI)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

SUBDIRS = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/firestation)) \
		  $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/loc_viewer)) \
		  $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/fuse_viewer)) \
		  $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/naocamtest_gui)) \
		  $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/colormap_editor))

LIBS_fvshmem = $(VISION_CAM_LIBS) stdc++ fawkescore fawkesutils fvutils
OBJS_fvshmem = shmem.o

LIBS_fvlistfwcams = $(VISION_CAM_LIBS) stdc++ fawkesutils fvcams
OBJS_fvlistfwcams = listfwcams.o

LIBS_fvlistsrcams = $(VISION_CAM_LIBS) stdc++ fawkesutils fvcams
OBJS_fvlistsrcams = listsrcams.o

LIBS_fvconverter = stdc++ fawkescore fawkesutils fvutils fvcams
OBJS_fvconverter = converter.o

LIBS_fvstereodecoder = $(VISION_CAM_LIBS) stdc++ fawkescore fawkesutils fvcams \
		       fvutils
OBJS_fvstereodecoder = stereodecoder.o

LIBS_fvtricalcdisp = $(VISION_CAM_LIBS) stdc++ fawkescore fawkesutils fvcams fvutils fvstereo
OBJS_fvtricalcdisp = tricalcdisp.o

LIBS_fvbb2gettric = $(VISION_CAM_LIBS) stdc++ fawkesutils fvcams fvutils
OBJS_fvbb2gettric = bb2gettric.o

LIBS_fvbb2rectlut = $(VISION_CAM_LIBS) stdc++ fawkescore fawkesutils \
		    fvcams fvutils fvstereo
OBJS_fvbb2rectlut = bb2rectlut.o

LIBS_fvbb2info = $(VISION_CAM_LIBS) stdc++ fawkescore fawkesutils fvcams \
		 fvutils fvstereo
OBJS_fvbb2info = bb2info.o

LIBS_fvviewer = $(VISION_CAM_LIBS) stdc++ fawkescore fawkesutils fvcams fvutils fvwidgets fvfilters
OBJS_fvviewer = viewer.o
CFLAGS_viewer    += $(CFLAGS_GTKMM) $(CFLAGS_SDL)
LDFLAGS_fvviewer += $(LDFLAGS_GTKMM) $(LDFLAGS_SDL)

LIBS_fvnet = stdc++ fawkescore fawkesutils fawkesnetcomm fvutils
OBJS_fvnet = net.o

LIBS_fvshowyuv = stdc++ fvutils fvwidgets
OBJS_fvshowyuv = show_yuv.o
CFLAGS_show_yuv += $(CFLAGS_SDL)
LDFLAGS_fvshowyuv += $(LDFLAGS_SDL)

LIBS_fvcmpp = stdc++ m fawkesutils fvutils
OBJS_fvcmpp = cmpp.o

OBJS_all = $(OBJS_fvshmem) $(OBJS_fvlistfwcams) $(OBJS_fvstereodecoder) \
	   $(OBJS_fvconverter) $(OBJS_fvtricalcdisp) $(OBJS_fvbb2gettric) \
	   $(OBJS_fvbb2info) $(OBJS_fvbb2rectlut) $(OBJS_fvviewer) \
	   $(OBJS_fvshowyuv) $(OBJS_fvnet) $(OBJS_fvcmpp) \
	   $(OBJS_fvlistsrcams)
BINS_all = $(BINDIR)/fvshmem $(BINDIR)/fvcmpp

ifneq ($(realpath $(SRCDIR)/net.cpp),)
  BINS_all += $(BINDIR)/fvnet
endif
ifneq ($(realpath $(SRCDIR)/bb2rectlut.cpp),)
  BINS_all += $(BINDIR)/fvbb2rectlut
endif
ifneq ($(realpath $(SRCDIR)/converter.cpp),)
  BINS_all += $(BINDIR)/fvconverter
endif
ifeq ($(HAVE_TRICLOPS_SDK),1)
  BINS_all += $(BINDIR)/fvtricalcdisp
else
  ifneq ($(realpath $(SRCDIR)/tricalcdisp.cpp),)
    WARN_TARGETS += warning_tricalcdisp
  endif
  ifneq ($(realpath $(SRCDIR)/bb2rectlut.cpp),)
    WARN_TARGETS += warning_bb2rectlut_triclops
  endif
endif

ifeq ($(HAVE_FIREWIRE_CAM),1)
  BINS_all += $(BINDIR)/fvlistfwcams
else
  ifneq ($(realpath $(SRCDIR)/listfwcams.cpp),)
    WARN_TARGETS += warning_listfwcams
  endif
endif

ifeq ($(HAVE_SWISSRANGER_CAM),1)
  BINS_all += $(BINDIR)/fvlistsrcams
else
  ifneq ($(realpath $(SRCDIR)/listsrcams.cpp),)
    WARN_TARGETS += warning_listsrcams
  endif
endif

ifeq ($(HAVE_BUMBLEBEE2_CAM),1)
  BINS_all += $(BINDIR)/fvbb2gettric $(BINDIR)/fvstereodecoder $(BINDIR)/fvbb2info
else
  ifneq ($(realpath $(SRCDIR)/bb2gettric.cpp),)
    WARN_TARGETS += warning_bb2gettric
  endif
  ifneq ($(realpath $(SRCDIR)/bb2info.cpp),)
    WARN_TARGETS += warning_bb2info
  endif
  ifneq ($(realpath $(SRCDIR)/stereodecoder.cpp),)
    WARN_TARGETS += warning_stereodecoder
  endif
  ifneq ($(realpath $(SRCDIR)/bb2rectlut.cpp),)
    WARN_TARGETS += warning_bb2rectlut_bb2
  endif
endif
ifeq ($(HAVE_SDL)$(HAVE_GTKMM),11)
  BINS_gui += $(BINDIR)/fvviewer
else
  WARN_TARGETS_GUI += warning_viewer
endif
ifeq ($(HAVE_SDL),1)
  BINS_gui += $(BINDIR)/fvshowyuv
else
  WARN_TARGETS_GUI += warning_showyuv
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
gui: $(WARN_TARGETS_GUI)

.PHONY: $(WARN_TARGETS)
warning_listfwcams:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building listfwcams$(TNORMAL) (no Firewire camera support)";
warning_listsrcams:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building listsrcams$(TNORMAL) (no SwissRanger camera support)";
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

include $(BUILDSYSDIR)/base.mk

