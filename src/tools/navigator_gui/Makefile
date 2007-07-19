#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator GUI
#                            -------------------
#   Created on Tue May 29 20:00:00 2007
#   copyright (C) 2007 by Martin Liebenberg, AllemaniACs RoboCup Team
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
#*****************************************************************************

BASEDIR = ../../..

CFLAGS = -g `pkg-config gtkmm-2.4 --cflags`


LIBS_navigator_gui = core utils netcomm navigator
LDFLAGS_navigator_gui = `pkg-config gtkmm-2.4 --libs`
OBJS_navigator_gui = navigator_gui.o

BINS_all = $(BINDIR)/navigator_gui


OBJS_all    = $(OBJS_navigator_gui)

include $(BASEDIR)/etc/buildsys/base.mk
