#*****************************************************************************
#           Makefile Build System for Fawkes: Configuration Utils
#                            -------------------
#   Created on Sun Jan 07 14:56:02 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

CFLAGS = -g $(shell $(PKGCONFIG) --cflags sqlite3)
LDFLAGS_libconfig = $(shell $(PKGCONFIG) --libs sqlite3)

LIBS_libconfig = stdc++ core netcomm
OBJS_libconfig =	\
		config.o		\
		netconf.o		\
		sqlite.o

OBJS_all = $(OBJS_libconfig)
LIBS_all  = $(LIBDIR)/libconfig.so

include $(BASEDIR)/etc/buildsys/base.mk

