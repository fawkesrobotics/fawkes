#*****************************************************************************
#         Makefile Build System for Fawkes: FireVision OmniBall Plugin
#                            -------------------
#   Created on Tue Jun 26 17:32:34 2007
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
#           $Id: Makefile 269 2007-06-27 14:25:06Z tim $
# last modified: $Date: 2007-06-27 16:25:06 +0200 (Mi, 27 Jun 2007) $
#            by: $Author: tim $
#
#*****************************************************************************

BASEDIR = ../../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS)
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_fvomniball = core utils fvutils fvcams fvmodels fvclassifiers
OBJS_fvomniball = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_fvomniball)
PLUGINS_all = $(PLUGINDIR)/fvomniball.so

include $(BASEDIR)/etc/buildsys/base.mk
