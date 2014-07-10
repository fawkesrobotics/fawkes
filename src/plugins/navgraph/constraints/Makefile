#*****************************************************************************
#              Makefile Build System for Fawkes: NavGraph Aspect
#                            -------------------
#   Created on Tue Oct 08 18:11:35 2013
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

LIBS_libfawkesnavgraphconstraints = fawkescore fawkesaspects fawkesutils
OBJS_libfawkesnavgraphconstraints = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesnavgraphconstraints)

LIBS_all = $(LIBDIR)/libfawkesnavgraphconstraints.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

