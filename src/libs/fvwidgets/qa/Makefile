#*****************************************************************************
#          Makefile Build System for Fawkes : FireVision Widgets QA
#                            -------------------
#   Created on Wed Apr 11 16:00:32 2007
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

OBJS_fv_qa_image_display := qa_image_display.o
LIBS_fv_qa_image_display := fvwidgets

OBJS_all = $(OBJS_fv_qa_image_display)

ifeq ($(HAVE_SDL),1)
  BINS_all = $(BINDIR)/fv_qa_image_display
  CFLAGS  += $(shell $(PKGCONFIG) --cflags 'sdl')
  LDFLAGS += $(shell $(PKGCONFIG) --libs 'sdl')
else
  WARN_TARGETS += warning_sdl
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
warning_sdl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting SDL QA Apps$(TNORMAL) (SDL[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

