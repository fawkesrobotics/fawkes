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

LIBS_fvstereodecoder = $(VISION_CAM_LIBS) utils fvcams
OBJS_fvstereodecoder = stereodecoder.o

LIBS_fvtricalcdisp = $(VISION_CAM_LIBS) utils fvcams fvstereo
OBJS_fvtricalcdisp = tricalcdisp.o

OBJS_all = $(OBJS_fvshmem) $(OBJS_listfwcams) $(OBJS_stereodecoder) \
	   $(OBJS_fvtricalcdisp) $(OBJS_fvbb2gettric)
BINS_all = $(BINDIR)/fvshmem

LIBS_fvbb2gettric = $(VISION_CAM_LIBS) utils fvcams fvutils
OBJS_fvbb2gettric = bb2gettric.o

ifeq ($(HAVE_TRICLOPS_SDK),1)
  BINS_all += $(BINDIR)/fvtricalcdisp
else
  WARN_TARGETS += warning_tricalcdisp
endif

ifeq ($(HAVE_FIREWIRE_CAM),1)
  BINS_all += $(BINDIR)/listfwcams
else
  WARN_TARGETS += warning_listfwcams
endif

ifeq ($(HAVE_BUMBLEBEE2_CAM),1)
  BINS_all += $(BINDIR)/fvbb2gettric $(BINDIR)/stereodecoder
else
  WARN_TARGETS += warning_bb2gettric
  WARN_TARGETS += warning_stereodecoder
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)

.PHONY: warning_listfwcams
warning_listfwcams:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building listfwcams$(TNORMAL) (no Firewire camera support)";
.PHONY: warning_stereodecoder
warning_stereodecoder:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building stereodecoder$(TNORMAL) (no Bumblebee2 camera support)";
.PHONY: warning_tricalcdisp
warning_tricalcdisp:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvtricalcdisp$(TNORMAL) (no Triclops SDK support)";
.PHONY: warning_bb2gettric
warning_bb2gettric:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvbb2gettric$(TNORMAL) (no Bumblebee2 camera support)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

