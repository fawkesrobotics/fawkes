#*****************************************************************************
#                 Makefile Build System for Fawkes: Navigator Plugin
#                            -------------------
#   Created on Tue May 29 18:41:17 2007
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

BASEDIR = ../../../..

CFLAGS = -g `gts-config --cflags`


LIBS_libgts_obstacle = core utils vmc netcomm
# gts
OBJS_libgts_obstacle = gts_obstacle.o
LDFLAGS_libgts_obstacle = `gts-config --libs`

OBJS_all    = $(OBJS_libgts_obstacle)
LIBS_all = $(LIBDIR)/libgts_obstacle.so

include $(BASEDIR)/etc/buildsys/base.mk
