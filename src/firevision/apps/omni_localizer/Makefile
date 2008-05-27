#*****************************************************************************
#         Makefile Build System for Fawkes: FireVision OmniLocalizer Plugin
#                            -------------------
#   Created on Tue Jun 26 17:32:34 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

HAVE_BOOST_RANDOM = $(if $(shell test -f /usr/include/boost/random.hpp; echo $${?/1/}),1,0)

ifneq ($(HAVE_BOOST_RANDOM),1)
  WARN_TARGETS += warning_boost_random
else
  CFLAGS   += $(VISION_CFLAGS)
  LDFLAGS  += $(VISION_LDFLAGS)
  INCDIRS  += $(VISION_INCDIRS)
  LIBDIRS  += $(VISION_LIBDIRS)
  LIBS     += $(VISION_LIBS)

  LIBS_fvomnilocalizer = fawkesaspects fvutils fvcams fvmodels fvclassifiers
  OBJS_fvomnilocalizer = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

  OBJS_all    = $(OBJS_fvomnilocalizer)
  PLUGINS_all = $(PLUGINDIR)/fvomnilocalizer.so
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_boost_random
warning_boost_random:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Localizer Plugin$(TNORMAL) (boost/random.hpp missing)"
endif

include $(BASEDIR)/etc/buildsys/base.mk

