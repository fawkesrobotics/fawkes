#*****************************************************************************
#              Makefile Build System for Fawkes: MongoDB Aspect
#                            -------------------
#   Created on Mon Dec 06 01:01:10 2010
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
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_libfawkesmongodbaspect = fawkescore fawkesaspects
OBJS_libfawkesmongodbaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

CFLAGS += $(CFLAGS_MONGODB)
LDFLAGS += $(LDFLAGS_MONGODB)

OBJS_all    = $(OBJS_libfawkesmongodbaspect)
LIBS_all    = $(LIBDIR)/libfawkesmongodbaspect.so
ifeq ($(HAVE_MONGODB),1)
  LIBS_build = $(LIBS_all)
endif

include $(BUILDSYSDIR)/base.mk
