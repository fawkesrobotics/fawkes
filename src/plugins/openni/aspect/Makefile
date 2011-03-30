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
include $(SRCDIR)/../openni.mk

CFLAGS  += $(CFLAGS_OPENNI)
LDFLAGS += $(LDFLAGS_OPENNI)

LIBS_libfawkesopenniaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesopenniaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesopenniaspect)

ifeq ($(HAVE_OPENNI),1)
  LIBS_all = $(LIBDIR)/libfawkesopenniaspect.so
else
  WARN_TARGETS += warning_openni
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openni
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenNI Aspect$(TNORMAL) (openni[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

