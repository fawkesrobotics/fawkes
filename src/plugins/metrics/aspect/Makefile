#*****************************************************************************
#             Makefile Build System for Fawkes: Metrics Aspect
#                            -------------------
#   Created on Fri Jul 28 20:05:22 2017
#   Copyright (C) 2006-2017 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_libfawkesmetricsaspect = stdc++ fawkescore fawkesaspects
OBJS_libfawkesmetricsaspect = metrics.o metrics_supplier.o metrics_inifin.o metrics_manager.o

OBJS_all    = $(OBJS_libfawkesmetricsaspect)

ifeq ($(HAVE_CPP14),1)
	LIBS_all = $(LIBDIR)/libfawkesmetricsaspect.so
else
	WARN_TARGETS += warning_cpp14
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_cpp14
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Metrics Aspect$(TNORMAL) (C++14 not supported)"
endif

include $(BUILDSYSDIR)/base.mk

