#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Cams QA
#                            -------------------
#   Created on Tue Apr 08 15:38:55 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

OBJS_fv_qa_fwcam := qa_fwcam.o
LIBS_fv_qa_fwcam := fvcams

OBJS_all = $(OBJS_fv_qa_fwcam)
BINS_all = $(BINDIR)/fv_qa_fwcam

include $(BUILDSYSDIR)/base.mk

