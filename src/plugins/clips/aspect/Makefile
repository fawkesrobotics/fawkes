#*****************************************************************************
#             Makefile Build System for Fawkes: CLIPS Aspect
#                            -------------------
#   Created on Sat Jun 16 14:27:41 2012 (Mexico City)
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/clips.mk

CFLAGS  += $(CFLAGS_CLIPS)
LDFLAGS += $(LDFLAGS_CLIPS)

LIBS_libfawkesclipsaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesclipsaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesclipsaspect)

ifeq ($(HAVE_CLIPS),1)
  LIBS_all = $(LIBDIR)/libfawkesclipsaspect.so
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Aspect$(TNORMAL) (CLIPS not found)"
endif

include $(BUILDSYSDIR)/base.mk

