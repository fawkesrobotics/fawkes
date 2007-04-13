#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Utils
#                            -------------------
#   Created on Sun Feb 04 22:32:29 2007
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

BASEDIR = ../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS)
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

# We are lazy in the utils...
OBJS_libfvutils := $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))
LIBS_libfvutils := core utils jpeg png

OBJS_all = $(OBJS_libfvutils)
LIBS_all = $(LIBDIR)/libfvutils.so

include $(BASEDIR)/etc/buildsys/base.mk

test:
	@echo $(OBJS_all)

