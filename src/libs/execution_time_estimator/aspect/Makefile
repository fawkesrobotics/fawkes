#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Wed 29 Apr 2020 15:39:53 CEST
#   Copyright (C) 2020 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

LIBS_libfawkes_execution_time_estimator_aspect = fawkescore fawkesaspects fawkes_execution_time_estimator
OBJS_libfawkes_execution_time_estimator_aspect = execution_time_estimator.o execution_time_estimator_inifin.o

OBJS_all = $(OBJS_libfawkes_execution_time_estimator_aspect)
LIBS_all = $(LIBDIR)/libfawkes_execution_time_estimator_aspect.so

ifneq ($(HAVE_CPP17)$(HAVE_CPP20),)
  ifeq ($(HAVE_CPP20),1)
    CFLAGS += $(CFLAGS_CPP20)
  else
    CFLAGS += $(CFLAGS_CPP17)
  endif
  LIBS_build = $(LIBS_all)
else
  WARN_TARGETS = warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: $(WARN_TARGETS)
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting execution time estimator aspect$(TNORMAL) (C++17 support required)"
endif

include $(BUILDSYSDIR)/base.mk
