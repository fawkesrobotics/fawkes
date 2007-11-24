#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Fountain
#                            -------------------
#   Created on Sun Jun 17 17:47:54 2007 (Starbucks)
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

LIBS_fvfountain = core utils fvutils netcomm

ifeq ($(BUILD_TYPE),rcsoftx)
  include $(BASEDIR)/src/fawkes/netcomm/netcomm.mk
  ifeq ($(realpath $(QTDIR)/include),)
    WARN_TARGETS += warning_rcxqt
  else
    OBJS_fvfountain  = rcxmain.o config.o
    LIBS_fvfountain += pthread rutils
    INCDIRS         += $(QTDIR)/include
    BINS_all         = $(BINDIR)/fvfountain
  endif
else
  OBJS_fvfountain  = fountain_thread.o fountain_plugin.o
  PLUGINS_all      = $(PLUGINDIR)/fvfountain.so
endif

ifneq ($(SRCDIR),.)
all: $(WARN_TARGETS)
.PHONY: warning_rcxqt
warning_rcxqt:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Cannot build Fountain$(TNORMAL) (qt-devel not installed)";
endif

OBJS_all = $(OBJS_fvfountain)

include $(BASEDIR)/etc/buildsys/base.mk

