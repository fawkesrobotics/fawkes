#*****************************************************************************
#        Makefile Build System for Fawkes : FireVision Classifiers
#                            -------------------
#   Created on Mon Mar 19 15:12:35 2007
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

BASEDIR = ../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS)
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

# We are lazy...
OBJS_libfvclassifiers := $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))
LIBS_libfvclassifiers := fvutils

OBJS_all = $(OBJS_libfvclassifiers)
LIBS_all = $(LIBDIR)/libfvclassifiers.so

include $(BASEDIR)/etc/buildsys/base.mk

