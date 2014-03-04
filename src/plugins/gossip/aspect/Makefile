#*****************************************************************************
#             Makefile Build System for Fawkes: Gossip Aspect
#                            -------------------
#   Created on Fri Feb 28 17:00:21 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libfawkesgossipaspect = stdc++ fawkescore fawkesaspects fawkesutils fawkesgossip
OBJS_libfawkesgossipaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesgossipaspect)
LIBS_all = $(LIBDIR)/libfawkesgossipaspect.$(SOEXT)

include $(BUILDSYSDIR)/base.mk

