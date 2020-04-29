#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Thu 23 Apr 2020 19:36:47 CEST
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


BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk

SUBDIRS = aspect

LIBS_libfawkes_execution_time_estimator = fawkescore
OBJS_libfawkes_execution_time_estimator = execution_time_estimator.o

OBJS_all = $(OBJS_libfawkes_execution_time_estimator)

LIBS_all = $(LIBDIR)/libfawkes_execution_time_estimator.so

ifeq ($(HAVE_CPP17),1)
  CFLAGS        += $(CFLAGS_CPP17)
  LIBS_build     = $(LIBS_all)
  PLUGINS_build  = $(PLUGINS_all)
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)

warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting exeuction time estimator$(TNORMAL) (C++17 not supported)"
endif

ifneq ($(MAKECMDGOALS),clean)
aspect: | $(LIBDIR)/libfawkes_execution_time_estimator.so
endif


include $(BUILDSYSDIR)/base.mk
