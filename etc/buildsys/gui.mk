#*****************************************************************************
#     Makefile Build System for Fawkes: Config Settings specific to GUI
#                            -------------------
#   Created on Mon Nov 26 14:50:34 2007
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

ifndef __gui_mk_
__gui_mk := 1

include $(BASEDIR)/etc/buildsys/config.mk
include $(BASEDIR)/etc/buildsys/ext/gmsl

PC_GTKMM    = gtkmm-2.4
PC_CAIROMM  = cairomm-1.0
PC_GLADEMM  = libglademm-2.4
PKG_GTKMM   = gtkmm24[-devel]
PKG_CAIROMM = cairomm[-devel]
PKG_GLADEMM = libglademm24[-devel]

ifneq ($(PKGCONFIG),)
  HAVE_GTKMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_GTKMM)'; echo $${?/1/}),1,0)
  HAVE_CAIROMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_CAIROMM)'; echo $${?/1/}),1,0)
  HAVE_GLADEMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_GLADEMM)'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_GTKMM),1)
  CFLAGS_GTKMM     = -DHAVE_GTKMM $(shell $(PKGCONFIG) --cflags '$(PC_GTKMM)')
  LDFLAGS_GTKMM    = $(shell $(PKGCONFIG) --libs '$(PC_GTKMM)')
else
  PKG_MISSING += $(PKG_GTKMM)
endif

ifeq ($(HAVE_CAIROMM),1)
  CFLAGS_CAIROMM   = -DHAVE_CAIROMM $(shell $(PKGCONFIG) --cflags '$(PC_CAIROMM)')
  LDFLAGS_CAIROMM  = $(shell $(PKGCONFIG) --libs '$(PC_CAIROMM)')
else
  PKG_MISSING += $(PKG_CAIROMM)
endif

ifeq ($(HAVE_GLADEMM),1)
  CFLAGS_GLADEMM   = -DHAVE_GLADEMM $(shell $(PKGCONFIG) --cflags '$(PC_GLADEMM)')
  LDFLAGS_GLADEMM  = $(shell $(PKGCONFIG) --libs '$(PC_GLADEMM)')
else
  PKG_MISSING += $(PKG_GLADEMM)
endif

ifneq ($(HAVE_GTKMM)$(HAVE_CAIROMM)$(HAVE_GLADEMM),111)
  GUI_ERROR = ($(call merge, and ,$(PKG_MISSING)) not installed)
endif

# Some default warnings that may be used
ifneq ($(SRCDIR),.)
.PHONY: warning_gtkmm warning_cairomm warning_glademm
warning_gtkmm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Gtk dependent GUI apps$(TNORMAL) $(ERROR_GTKMM)";
warning_cairomm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Cairo dependent GUI apps$(TNORMAL) $(ERROR_CAIROMM)";
warning_glademm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Glade dependent GUI apps$(TNORMAL) $(ERROR_GLADEMM)";
endif

endif # __gui_mk_

