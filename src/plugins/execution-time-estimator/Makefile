#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Thu 23 Apr 2020 16:53:02 CEST
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
include $(LIBSRCDIR)/utils/utils.mk

LIBS_execution_time_estimator = fawkescore fawkes_execution_time_estimator
LIBS_libfawkes_execution_time_estimator = m fawkescore fawkesaspects

OBJS_execution_time_estimator = execution_time_estimator_plugin.o \
                                execution_time_estimator_thread.o \
                                estimators/config_estimator.o
OBJS_libfawkes_execution_time_estimator = execution_time_estimator.o \
                                execution_time_estimator_aspect/execution_time_estimator_aspect.o \
                                execution_time_estimator_aspect/execution_time_estimator_aspect_inifin.o \

OBJS_all = $(OBJS_execution_time_estimator) $(OBJS_libfawkes_execution_time_estimator)
PLUGINS_all = $(PLUGINDIR)/execution-time-estimator.so
LIBS_all = $(LIBDIR)/libfawkes_execution_time_estimator.so

ifeq ($(HAVE_CPP17),1)
  CFLAGS        += $(CFLAGS_CPP17)
  LIBS_build     = $(LIBS_all)
  PLUGINS_build  = $(PLUGINS_all)
else
  WARN_TARGETS += warning_cpp17
endif

$(PLUGINS_build): | $(LIBDIR)/libfawkes_execution_time_estimator.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)

warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting skiller simulator$(TNORMAL) (C++17 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
