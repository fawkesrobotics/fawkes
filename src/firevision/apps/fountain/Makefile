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

LIBS_fvfountain = core utils fvutils fvcams fvmodels fvclassifiers fvfilters netcomm

ifeq ($(BUILD_TYPE),rcsoftx)
  OBJS_fvfountain = fountain.o mini_image_producer.o rcxmain.o
  LIBS_fvfountain += pthread
  BINS_all       = $(BINDIR)/fvfountain
else
  OBJS_fvfountain = fountain_thread.o fountain_plugin.o
  PLUGINS_all = $(PLUGINDIR)/fvfountain.so
endif

OBJS_all = $(OBJS_fvfountain)

include $(BASEDIR)/etc/buildsys/base.mk

