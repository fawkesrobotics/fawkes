#*****************************************************************************
#              Makefile Build System for Fawkes: NaoQi Aspect
#                            -------------------
#   Created on Thu May 12 15:46:30 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(SRCDIR)/../nao.mk

CFLAGS  += $(CFLAGS_NAOQI)
LDFLAGS += $(LDFLAGS_NAOQI)

LIBS_libfawkesnaoqiaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesnaoqiaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesnaoqiaspect)

ifeq ($(HAVE_NAOQI),1)
  LIBS_all = $(LIBDIR)/libfawkesnaoqiaspect.so
else
  WARN_TARGETS += warning_naoqi
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_naoqi
warning_naoqi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting NaoQi Aspect$(TNORMAL) (NaoQi not found)"
endif

include $(BUILDSYSDIR)/base.mk

