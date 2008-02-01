#*****************************************************************************
#          Makefile Build System for Fawkes : FireVision Cannikin
#                            -------------------
#   Created on Mon Apr 09 12:13:49 2007 (Easter Monday)
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

# -Wno-non-virtual-dtor needed to stop gcc from throwing Qt warnings
CFLAGS   += $(VISION_CFLAGS) -Wno-non-virtual-dtor
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS) $(QTDIR)/include
LIBDIRS  += $(VISION_LIBDIRS) $(QTDIR)/lib
LIBS     += $(VISION_LIBS)

ifneq ($(HAVE_TRICLOPS_SDK),1)
  WARN_TARGETS += warning_triclops
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_triclops
warning_triclops:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Cannikin can only run in camless-mode$(TNORMAL) (Triclops SDK not available)"
endif

OBJS_cannikin = bbclient.o config.o main.o pipeline.o
LIBS_cannikin = core utils rutils fvutils fvcams fvmodels fvclassifiers fvstereo bb interfaces qt-mt

OBJS_bbcannikin = bbcannikin.o
LIBS_bbcannikin = core utils rutils bb interfaces qt-mt

OBJS_all = $(OBJS_cannikin) $(OBJS_bbcannikin)
BINS_all = $(BINDIR)/cannikin $(BINDIR)/bbcannikin

include $(BASEDIR)/etc/buildsys/base.mk

