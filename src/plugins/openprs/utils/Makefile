#*****************************************************************************
#             Makefile Build System for Fawkes: OpenPRS Utils
#                            -------------------
#   Created on Wed Aug 20 18:19:01 2014
#   Copyright (C) 2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/openprs.mk

HAVE_LIBDAEMON = $(if $(shell $(PKGCONFIG) --exists 'libdaemon'; echo $${?/1/}),1,0)
ifeq ($(HAVE_LIBDAEMON),1)
  CFLAGS_LIBDAEMON  += -DHAVE_LIBDAEMON $(shell $(PKGCONFIG) --cflags 'libdaemon')
  LDFLAGS_LIBDAEMON += $(shell $(PKGCONFIG) --libs 'libdaemon')
else
  WARN_TARGETS += warning_libdaemon
endif

LIBS_libfawkesopenprsutils = stdc++ pthread fawkescore fawkesutils
OBJS_libfawkesopenprsutils = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesopenprsutils)

ifeq ($(HAVE_OPENPRS),1)
  CFLAGS  += $(CFLAGS_OPENPRS)  $(CFLAGS_OPENPRS_MP)  $(CFLAGS_LIBDAEMON)
  LDFLAGS += $(LDFLAGS_OPENPRS) $(LDFLAGS_OPENPRS_MP) $(LDFLAGS_LIBDAEMON)

  LIBS_all = $(LIBDIR)/libfawkesopenprsutils.so
else
  WARN_TARGETS += warning_openprs
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openprs warning_libdaemon
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS utils$(TNORMAL) ($(OPENPRS_ERROR))"
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Sub-process cannot close all file handles$(TNORMAL) (libdaemon not found)"
endif

include $(BUILDSYSDIR)/base.mk

