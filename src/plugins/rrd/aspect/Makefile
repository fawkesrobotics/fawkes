#*****************************************************************************
#                Makefile Build System for Fawkes: RRD Aspect
#                            -------------------
#   Created on Fri Dec 17 00:18:29 2010
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(LIBSRCDIR)/utils/utils.mk

LIBS_libfawkesrrdaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesrrdaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesrrdaspect)

LIBS_all = $(LIBDIR)/libfawkesrrdaspect.so

include $(BUILDSYSDIR)/base.mk

