#*****************************************************************************
#               Makefile Build System for Fawkes: SyncPoint
#                            -------------------
#   Created on Thu Jan 23 15:17:42 2014
#   Copyright (C) 2014 by Till Hofmann
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
include $(BASEDIR)/etc/buildsys/config.mk


LIBS_libfawkessyncpoint = fawkescore 
OBJS_libfawkessyncpoint = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libfawkessyncpoint = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

#CFLAGS += $(CFLAGS_LUA)
#LDFLAGS_libfawkessyncpoint += $(LDFLAGS_LUA)

OBJS_all = $(OBJS_libfawkessyncpoint)

LIBS_all = $(LIBDIR)/libfawkessyncpoint.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
endif

include $(BUILDSYSDIR)/base.mk
