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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#

BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/gui.mk
#include $(BASEDIR)/src/firevision/fvconf.mk
#include $(BASEDIR)/src/firevision/tools/image_viewer/gtkconf.mk

CFLAGS   = $(VISION_CFLAGS) -Wno-packed -Wno-attributes -Wno-non-virtual-dtor
CFLAGS  += $(CFLAGS_GTKMM) $(CFLAGS_GLADEMM)
LDFLAGS  = $(VISION_LDFLAGS) $(LDFLAGS_GTKMM) $(LDFLAGS_GLADEMM)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_firestation = utils fvutils fvmodels fvclassifiers geometry fvcams
OBJS_firestation = color_train.o	\
		   mirror_calib.o	\
	           firestation.o	\
		   main.o
OBJS_all += $(OBJS_firestation)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM),11)
  BINS_gui += $(BINDIR)/firestation
else
  WARN_TARGETS += warning_firestation
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_firestation:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building shm_image_viewer$(TNORMAL) ($(GTK_CONF_ERR) not installed)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

