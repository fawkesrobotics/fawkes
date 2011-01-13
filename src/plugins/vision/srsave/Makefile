#*****************************************************************************
#     Makefile Build System for Fawkes: FireVision SwissRanger Save Plugin
#                            -------------------
#   Created on Fri Jan 22 10:46:09 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_fvsrsave = fawkescore fawkesaspects fvutils fvcams fvmodels fvclassifiers fvfilters
OBJS_fvsrsave = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_fvsrsave)
PLUGINS_all = $(PLUGINDIR)/fvsrsave.so

include $(BUILDSYSDIR)/base.mk

