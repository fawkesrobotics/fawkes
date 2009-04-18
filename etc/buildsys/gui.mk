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

PC_GTKMM        = gtkmm-2.4
PC_CAIROMM      = cairomm-1.0
PC_GLADEMM      = libglademm-2.4
PC_HILDONMM     = hildonmm
PC_HILDONFMMM   = hildon-fmmm
PC_GCONFMM      = gconfmm-2.6
PC_GLIBMM       = glibmm-2.4
PC_GTHREAD      = gthread-2.0
PKG_GTKMM       = gtkmm24[-devel]
PKG_CAIROMM     = cairomm[-devel]
PKG_GLADEMM     = libglademm24[-devel]
PKG_GCONFMM     = gconfmm26[-devel]
PKG_GLIBMM      = glibmm24[-devel]
PKG_GTHREAD     = glib2-devel

ifneq ($(PKGCONFIG),)
  HAVE_GTKMM   = $(if $(shell $(PKGCONFIG) --exists '$(PC_GTKMM)'; echo $${?/1/}),1,0)
  HAVE_CAIROMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_CAIROMM)'; echo $${?/1/}),1,0)
  HAVE_GLADEMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_GLADEMM)'; echo $${?/1/}),1,0)
  HAVE_GCONFMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_GCONFMM)'; echo $${?/1/}),1,0)
  HAVE_GLIBMM  = $(if $(shell $(PKGCONFIG) --exists '$(PC_GLIBMM)'; echo $${?/1/}),1,0)
  HAVE_GTHREAD = $(if $(shell $(PKGCONFIG) --exists '$(PC_GTHREAD)'; echo $${?/1/}),1,0)
  ifeq ($(BUILD_TYPE), maemo)
    HAVE_HILDONMM   = $(if $(shell $(PKGCONFIG) --exists '$(PC_HILDONMM)'; echo $${?/1/}),1,0)
    HAVE_HILDONFMMM = $(if $(shell $(PKGCONFIG) --exists '$(PC_HILDONFMMM)'; echo $${?/1/}),1,0)
  endif
endif

ifeq ($(HAVE_HILDONMM)$(HAVE_HILDONFMMM),11)
  CFLAGS_HILDONMM  = $(shell $(PKGCONFIG) --cflags '$(PC_HILDONMM)' '$(PC_HILDONFMMM)')
  LDFLAGS_HILDONMM = $(shell $(PKGCONFIG) --libs '$(PC_HILDONMM)' '$(PC_HILDONFMMM)')

  CFLAGS_GUI  += $(CFLAGS_HILDONMM)
  LDFLAGS_GUI += $(LDFLAGS_HILDONMM)
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

ifeq ($(HAVE_GCONFMM),1)
  CFLAGS_GCONFMM  = $(shell $(PKGCONFIG) --cflags '$(PC_GCONFMM)') -DHAVE_GCONFMM
  LDFLAGS_GCONFMM = $(shell $(PKGCONFIG) --libs '$(PC_GCONFMM)')
else
  PKG_MISSING += $(PKG_GCONFMM)
endif

ifeq ($(HAVE_GLIBMM),1)
  CFLAGS_GLIBMM  = $(shell $(PKGCONFIG) --cflags '$(PC_GLIBMM)') -DHAVE_GLIBMM
  LDFLAGS_GLIBMM = $(shell $(PKGCONFIG) --libs '$(PC_GLIBMM)')
else
  PKG_MISSING += $(PKG_GLIBMM)
endif

ifeq ($(HAVE_GTHREAD),1)
  CFLAGS_GTHREAD  = $(shell $(PKGCONFIG) --cflags '$(PC_GTHREAD)') -DHAVE_GTHREAD
  LDFLAGS_GTHREAD = $(shell $(PKGCONFIG) --libs '$(PC_GTHREAD)')
else
  PKG_MISSING += $(PKG_GTHREAD)
endif

ifeq ($(HAVE_GTKMM)$(HAVE_CAIROMM)$(HAVE_GLADEMM)$(HAVE_GTHREAD),1111)
  HAVE_GUI     = 1
  CFLAGS_GUI  += $(CFLAGS_GTKMM) $(CFLAGS_CAIROMM) $(CFLAGS_GLADEMM)	\
		$(CFLAGS_GCONFMM) $(CFLAGS_GTHREAD)
  LDFLAGS_GUI += $(LDFLAGS_GTKMM) $(LDFLAGS_CAIROMM) $(LDFLAGS_GLADEMM) \
		$(LDFLAGS_GCONFMM) $(LDFLAGS_GTHREAD)
else
  GUI_ERROR = ($(call merge, and ,$(PKG_MISSING)) not installed)
endif

# Some default warnings that may be used
ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_gtkmm warning_cairomm warning_glademm warning_hildonmm
warning_gtkmm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Gtk dependent GUI apps$(TNORMAL) $(ERROR_GTKMM)";
warning_cairomm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Cairo dependent GUI apps$(TNORMAL) $(ERROR_CAIROMM)";
warning_glademm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Glade dependent GUI apps$(TNORMAL) $(ERROR_GLADEMM)";
warning_gconfmm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting GConf dependent GUI apps$(TNORMAL) $(ERROR_GCONFMM)";
warning_gthread:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting GThread dependent GUI apps$(TNORMAL) $(ERROR_GTHREAD)";
warning_hildonmm:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting HildonMM dependent GUI apps$(TNORMAL) (hildonmm/hildon-fmmm not installed)";
endif

endif # __gui_mk_

