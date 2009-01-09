#*****************************************************************************
#            Makefile Build System for Fawkes : fuse_viewer
#                            -------------------
#   Created on Thu Dec 18 14:16:23 2008
#   copyright (C) 2008  Christof Rath <c.rath@student.tugraz.at>
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

LIBS_fuse_viewer = fawkescore fawkesnetcomm fawkesplugin fawkesutils fawkesguiutils fvwidgets fvcams
OBJS_fuse_viewer = $(patsubst %.cpp,%.o,$(patsubst cox/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
OBJS_all += $(OBJS_fuse_viewer)

ifeq ($(HAVE_GTKMM)$(HAVE_GLADEMM)$(HAVE_AVAHI),111)
  BINS_gui += $(BINDIR)/fuse_viewer
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
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fuse_viewer$(TNORMAL) $(GUI_ERROR)";
warning_avahi:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Not building fuse_viewer$(TNORMAL) (Avahi not installed)";
endif

include $(BASEDIR)/etc/buildsys/base.mk

