#*****************************************************************************
#         Makefile Build System for Fawkes: FireVision Retriever Plugin
#                            -------------------
#   Created on Tue Jun 26 17:32:34 2007
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

ifeq ($(BUILD_TYPE),rcsoftx)
  LIBS_fvretriever = fvutils fvcams
  OBJS_fvretriever = rcxretriever.o
  BINS_all = $(BINDIR)/fvretriever
else
  LIBS_fvretriever = fawkescore fawkesutils fawkesaspects fvutils fvcams fvmodels
  OBJS_fvretriever = fvretriever_plugin.o retriever_thread.o
  PLUGINS_all = $(PLUGINDIR)/fvretriever.so
endif

OBJS_all    = $(OBJS_fvretriever)

include $(BUILDSYSDIR)/base.mk

