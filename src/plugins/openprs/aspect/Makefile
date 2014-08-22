#*****************************************************************************
#             Makefile Build System for Fawkes: OpenPRS Aspects
#                            -------------------
#   Created on Mon Aug 18 14:18:01 2014
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

LIBS_libfawkesopenprsaspect = stdc++ fawkescore fawkesaspects fawkesutils \
			      fawkesopenprsutils
OBJS_libfawkesopenprsaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesopenprsaspect)

ifeq ($(HAVE_OPENPRS),1)
  CFLAGS  += $(CFLAGS_OPENPRS)  $(CFLAGS_OPENPRS_MP)
  LDFLAGS += $(LDFLAGS_OPENPRS) $(LDFLAGS_OPENPRS_MP)

  LIBS_all = $(LIBDIR)/libfawkesopenprsaspect.so
else
  WARN_TARGETS += warning_openprs
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openprs
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Aspects$(TNORMAL) ($(OPENPRS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk

