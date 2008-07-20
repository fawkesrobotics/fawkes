#*****************************************************************************
#          Makefile Build System for Fawkes : FireVision PanTilter
#                            -------------------
#   Created on Sun Apr 20 23:18:56 2008 (GO2008, day 2)
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id: Makefile 949 2008-04-17 12:35:32Z stf $
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

OBJS_pantilter = bbclient.o main.o
LIBS_pantilter = fawkescore fawkesutils utils fvutils fvcams fvmodels fvclassifiers bb interfaces qt-mt fvfilters pthread

OBJS_all = $(OBJS_pantilter)
BINS_all = $(BINDIR)/pantilter

include $(BASEDIR)/etc/buildsys/base.mk

