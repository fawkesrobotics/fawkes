#*****************************************************************************
#         Makefile Build System for Fawkes: FireVision Facer Plugin
#                            -------------------
#   Created on Apr Sat 19 12:34:48 2008 (on the way to German Open 2008, Hannover)
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_fvfacer = fawkesaspects fvutils fvcams fvmodels
OBJS_fvfacer = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_fvfacer)
PLUGINS_all = $(PLUGINDIR)/fvfacer.so

include $(BASEDIR)/etc/buildsys/base.mk

