#*****************************************************************************
#           Makefile Build System for Fawkes : FireVision Widgets
#                            -------------------
#   Created on Mon Nov 05 17:35:24 2007
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

BASEDIR = ../../..
include $(BASEDIR)/src/firevision/fvconf.mk
include $(BASEDIR)/etc/buildsys/gui.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

ifeq ($(HAVE_SDL),1)
  LIBS_gui = $(LIBDIR)/libfvwidgets.so
  CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_SDL)
  LDFLAGS += $(LDFLAGS_GTKMM) $(LDFLAGS_SDL)
else
  WARN_TARGETS += warning_sdl
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)
warning_sdl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting SDL Widgets$(TNORMAL) (SDL[-devel] not installed)"
endif

OBJS_libfvwidgets := $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
LIBS_libfvwidgets := fawkescore fawkesutils fvutils

OBJS_all = $(OBJS_libfvwidgets)

include $(BASEDIR)/etc/buildsys/base.mk

