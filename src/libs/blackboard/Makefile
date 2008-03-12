#*****************************************************************************
#               Makefile Build System for Fawkes : BlackBoard
#                            -------------------
#   Created on Wed Oct 04 10:49:27 2006
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libblackboard = core utils interface netcomm
OBJS_libblackboard = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all =	$(OBJS_libblackboard) $(OBJS_blackboard)
LIBS_all =	$(LIBDIR)/libblackboard.so

include $(BASEDIR)/etc/buildsys/base.mk

