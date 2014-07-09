#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Models
#                            -------------------
#   Created on Thu Mar 15 14:20:34 2007
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

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDCONFDIR)/tf/tf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_libfvmodels = m fawkescore fawkesutils fvutils fvfilters
HDRS_libfvmodels = $(patsubst %.o,%.h,$(OBJS_libfvmodels))

ifneq ($(HAVE_SHAPE_MODELS),1)
  MODELS_FILTEROUT += shape/%
endif

ifeq ($(HAVE_IPP)$(HAVE_OPENCV),01)
  CFLAGS  += $(CFLAGS_OPENCV)
  LDFLAGS += $(LDFLAGS_OPENCV)
endif

ifeq ($(HAVE_IPP)$(HAVE_OPENCV),00)
  MODELS_FILTEROUT += mirror/mirror_calib%
endif

ifeq ($(HAVE_TF),1)
  CFLAGS += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  LIBS_libfvmodels += fawkestf
else
  WARN_TARGETS += warning_tf
  MODELS_FILTEROUT += relative_position/position_to_pixel.%
endif


OBJS_libfvmodels := $(patsubst %.cpp,%.o,$(filter-out $(MODELS_FILTEROUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))

OBJS_all = $(OBJS_libfvmodels)
LIBS_all = $(LIBDIR)/libfvmodels.so

include $(BUILDSYSDIR)/base.mk

