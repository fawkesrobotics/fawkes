#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Fountain
#                            -------------------
#   Created on Sun Jun 17 17:47:54 2007 (Starbucks)
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
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS)
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

OBJS_fountain = fountain.o mini_image_producer.o
LIBS_fountain = core utils fvutils fvcams fvmodels fvclassifiers fvfilters netcomm

ifeq ($(BUILD_TYPE),rcsoftx)
  OBJS_fountain += rcxmain.o bbclient.o config.o
  LIBS_fountain += rutils bb interfaces qt-mt pthread
  BINS_all       = $(BINDIR)/fountain
  # -Wno-non-virtual-dtor needed to stop gcc from throwing Qt warnings
  CFLAGS        += -Wno-non-virtual-dtor
  INCDIRS       += $(QTDIR)/include
  LIBDIRS       += $(QTDIR)/lib
else
  OBJS_fountain += fawkes_plugin.o
#  PLUGINS_all = $(PLUGINDIR)/fountain.so
endif

OBJS_all = $(OBJS_fountain)

include $(BASEDIR)/etc/buildsys/base.mk

