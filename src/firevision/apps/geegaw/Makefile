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

OBJS_geegaw = bbclient.o config.o main.o pipeline.o
LIBS_geegaw = core utils rutils fvutils fvcams fvmodels fvclassifiers bb interfaces qt-mt fvfilters pthread

OBJS_bbgeegaw = bbgeegaw.o
LIBS_bbgeegaw = core utils rutils bb interfaces

OBJS_all = $(OBJS_geegaw) $(OBJS_bbgeegaw)
BINS_all = $(BINDIR)/geegaw $(BINDIR)/bbgeegaw

include $(BASEDIR)/etc/buildsys/base.mk

