#*****************************************************************************
#            Makefile Build System for Fawkes : Firestation
#                            -------------------
#   Created on Fri Dec 28 10:00:28 2007
#   copyright (C) 2007 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/etc/buildsys/gui.mk
include $(LIBSRCDIR)/netcomm/netcomm.mk

CFLAGS   = $(VISION_CFLAGS)
CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS  = $(VISION_LDFLAGS) $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_yuv_viewer = fawkescore fvutils fvwidgets
OBJS_yuv_viewer = $(patsubst %.cpp,%.o,$(patsubst cox/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
OBJS_all += $(OBJS_yuv_viewer)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui += $(BINDIR)/yuv_viewer
else
  WARN_TARGETS += warning_guilibs
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_guilibs:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building yuv_viewer$(TNORMAL) $(GUI_ERROR)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

