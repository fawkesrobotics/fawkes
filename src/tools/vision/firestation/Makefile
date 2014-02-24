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

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDSYSDIR)/gui.mk
include $(BUILDCONFDIR)/netcomm/netcomm.mk

CFLAGS   = $(VISION_CFLAGS)
CFLAGS  += $(CFLAGS_GUI)
LDFLAGS  = $(VISION_LDFLAGS) $(LDFLAGS_GUI)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_firestation = stdc++ m fawkescore fawkesutils fvutils \
		   fawkesnetcomm fvmodels fvcams fvwidgets \
		   fawkesguiutils
ifeq ($(HAVE_BULB_CREATOR),1)
  LIBS_firestation += fvbulbcalib
endif
OBJS_firestation = color_train_widget.o         \
		   fuse_transfer_widget.o	\
		   colormap_viewer_widget.o	\
	           firestation.o		\
		   main.o
OBJS_all += $(OBJS_firestation)

ifeq ($(HAVE_GUI)$(HAVE_AVAHI),11)
  BINS_gui += $(BINDIR)/firestation
  INST_RESDIRS = guis/firestation
  MANPAGES_gui += $(MANDIR)/man1/firestation.1
else
  ifneq ($(HAVE_AVAHI),1)
    WARN_TARGETS += warning_avahi
  else
    WARN_TARGETS += warning_guilibs
  endif
endif

ifneq ($(HAVE_IPP)$(HAVE_OPENCV),00)
  CFLAGS  += $(CFLAGS_OPENCV) -DHAVE_MIRROR_CALIB
  LDFLAGS += $(LDFLAGS_OPENCV)
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
gui: $(WARN_TARGETS)
  endif

.PHONY: warning_guilibs warning_avahi
warning_guilibs:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building firestation$(TNORMAL) $(GUI_ERROR)";
warning_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building firestation$(TNORMAL) (Avahi not installed)";
endif

include $(BUILDSYSDIR)/base.mk

