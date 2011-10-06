#*****************************************************************************
#        Makefile Build System for Fawkes : FireVision FUSE Viewer
#                            -------------------
#   Created on Thu Dec 18 14:16:23 2008
#   Copyright (C) 2008  Christof Rath <c.rath@student.tugraz.at>
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

CFLAGS   = $(VISION_CFLAGS) $(CFLAGS_GUI)
LDFLAGS  = $(VISION_LDFLAGS) $(LDFLAGS_GUI)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_fvfuseviewer = stdc++ fawkescore fawkesnetcomm fawkesplugin fawkesutils \
		    fawkesguiutils fvwidgets fvcams
OBJS_fvfuseviewer = $(patsubst %.cpp,%.o,$(patsubst cox/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
OBJS_all += $(OBJS_fvfuseviewer)

ifeq ($(HAVE_GUI),1)
  BINS_gui += $(BINDIR)/fvfuseviewer
  INST_RESDIRS = guis/fuse_viewer
  MANPAGES_all += $(MANDIR)/man1/fvfuseviewer.1
else
  ifneq ($(HAVE_AVAHI),1)
    WARN_TARGETS += warning_avahi
  else
    WARN_TARGETS += warning_guilibs
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
gui: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_guilibs:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvfuseviewer$(TNORMAL) $(GUI_ERROR)";
warning_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fvfuseviewer$(TNORMAL) (Avahi not installed)";
endif

include $(BUILDSYSDIR)/base.mk

