#*****************************************************************************
#             Makefile Build System for Fawkes: ASP Aspect
#                            -------------------
#   Created on Thu Oct 20 15:00:34 2016
#   Copyright (C) 2016 by Björn Schäpers
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
include $(SRCDIR)/../clingo.mk

LIBS_libfawkesaspaspect = stdc++ fawkescore fawkesaspects fawkesutils pthread
OBJS_libfawkesaspaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all = $(OBJS_libfawkesaspaspect)
LIBS_all = $(LIBDIR)/libfawkesaspaspect.so

ifeq ($(HAVE_CPP11),1)
  ifeq ($(HAVE_CLINGO),1)
    CFLAGS  += $(CFLAGS_CLINGO)  $(CFLAGS_CPP11)
    LDFLAGS += $(LDFLAGS_CLINGO) $(LDFLAGS_CPP11)

    LIBS_build = $(LIBS_all)
  else
    WARN_TARGETS += warning_clingo
  endif
else
  WARN_TARGETS += warning_cpp11
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clingo warning_cpp11
warning_clingo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP Aspect$(TNORMAL) ($(CLINGO_ERROR))"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ASP Aspect$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk

