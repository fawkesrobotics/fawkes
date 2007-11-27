#*****************************************************************************
#            Makefile Build System for Fawkes : GTK Config
#                            -------------------
#   Created on Tue Nov 06 10:30:56 2007
#   copyright (C) 2006 by Tim Niemueller, AllemaniACs RoboCup Team
#                 2007 by Daniel Beck, AllemaniACs RoboCup Team
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

include $(BASEDIR)/etc/buildsys/config.mk

DEPENDENCIES = GTKMM-2.4 LIBGLADEMM-2.4

# check for gtkmm-2.4 & libglademm-2.4
ifneq ($(PKGCONFIG),)
  HAVE_LIBGLADEMM-2.4 = $(if $(shell $(PKGCONFIG) --exists 'libglademm-2.4'; echo $${?/1}),1,0)
  HAVE_GTKMM-2.4 = $(if $(shell $(PKGCONFIG) --exists 'gtkmm-2.4'; echo $${?/1}),1,0)
endif

ifeq ($(HAVE_GTKMM-2.4),1)
  GTK_CFLAGS += $(shell $(PKGCONFIG) gtkmm-2.4 --cflags) 
  GTK_LDFLAGS += $(shell $(PKGCONFIG) gtkmm-2.4 --libs)
else
  GTK_CONF_ERR += "gtkmm-2.4"
endif

ifeq ($(HAVE_LIBGLADEMM-2.4),1)
  GTK_CFLAGS += $(shell $(PKGCONFIG) libglademm-2.4 --cflags)
  GTK_LDFLAGS += $(shell $(PKGCONFIG) libglademm-2.4 --libs)
else
  GTK_CONF_ERR += "libglademm-2.4"
endif

ifeq ($(MAKECMDGOALS),printdeps)
DEPENDENCIES_PRINT = $(foreach DEP,$(DEPENDENCIES),"\b  "$(DEP): $(if $(subst 0,,$(HAVE_$(DEP))),"yes","no")"\n")
printdeps:
	$(SILENT)echo "GTK dependencies:"
	$(SILENT)echo -e $(DEPENDENCIES_PRINT)
endif