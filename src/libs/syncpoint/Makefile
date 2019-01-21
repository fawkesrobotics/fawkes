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

LIBS_libfawkessyncpoint = fawkescore fawkesutils
OBJS_libfawkessyncpoint = $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))
HDRS_libfawkessyncpoint = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkessyncpoint)
LIBS_all = $(LIBDIR)/libfawkessyncpoint.so
LIBS_build = $(LIBS_all)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
endif

include $(BUILDSYSDIR)/base.mk
