#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Models
#                            -------------------
#   Created on Thu Mar 15 14:20:34 2007
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

ifneq ($(HAVE_SHAPE_MODELS),1)
  MODELS_FILTEROUT += shape/%
endif

# We are lazy in the models...
OBJS_libfvmodels := $(patsubst %.cpp,%.o,$(filter-out $(MODELS_FILTEROUT),$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))
LIBS_libfvutils  := utils fvutils

OBJS_all = $(OBJS_libfvmodels)
LIBS_all = $(LIBDIR)/libfvmodels.so

include $(BASEDIR)/etc/buildsys/base.mk

