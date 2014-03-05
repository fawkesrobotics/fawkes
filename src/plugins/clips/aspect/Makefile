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

LIBS_libfawkesclipsaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesclipsaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesclipsaspect)

ifeq ($(HAVE_CPP11)$(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_CLIPS)

  LIBS_all = $(LIBDIR)/libfawkesclipsaspect.so
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_cpp11
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Aspect$(TNORMAL) (CLIPS not found)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build CLIPS aspect$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk

