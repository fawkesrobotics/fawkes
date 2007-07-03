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

OBJS_all = $(OBJS_fvshmem) $(OBJS_listfwcams)
BINS_all = $(BINDIR)/fvshmem

ifeq ($(HAVE_FIREWIRE_CAM),1)
  BINS_all += $(BINDIR)/listfwcams
else
  WARN_TARGETS += warning_listfwcams
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)

.PHONY: warning_listfwcams
warning_listfwcams:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building listfwcams$(TNORMAL) (no Firewire camera support)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

