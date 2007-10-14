#*****************************************************************************
#              Makefile Build System for Fawkes: Geometry Library
#                            -------------------
#   Created on Sun Oct 14 18:48:54 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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

BASEDIR = ../../..

LIBS_libgeometry = stdc++ core
OBJS_libgeometry = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))

OBJS_all = $(OBJS_libgeometry)
LIBS_all  = $(LIBDIR)/libgeometry.so

include $(BASEDIR)/etc/buildsys/base.mk

