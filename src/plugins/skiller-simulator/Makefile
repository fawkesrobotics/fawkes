#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Mon 06 May 2019 08:54:03 CEST
#   Copyright (C) 2019 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

LIBS_skiller_simulator  = fawkescore fawkesutils fawkesaspects \
                          fawkesblackboard fawkesinterface fawkeslogging \
                          fawkes_skiller_time_estimator \
                          SkillerInterface
LIBS_libfawkes_skiller_time_estimator = m fawkescore

OBJS_skiller_simulator  = skiller_simulator_plugin.o exec_thread.o \
                          execution_time_estimator_aspect/execution_time_estimator_aspect.o \
                          execution_time_estimator_aspect/execution_time_estimator_aspect_inifin.o \
                          estimators/config_estimator.o
OBJS_libfawkes_skiller_time_estimator = execution_time_estimator.o

OBJS_all                = $(OBJS_skiller_simulator) $(OBJS_libfawkes_skiller_time_estimator)
PLUGINS_all             = $(PLUGINDIR)/skiller-simulator.so
LIBS_all                = $(LIBDIR)/libfawkes_skiller_time_estimator.so

ifeq ($(HAVE_CPP17),1)
  CFLAGS        += $(CFLAGS_CPP17)
  LIBS_build     = $(LIBS_all)
  PLUGINS_build  = $(PLUGINS_all)
else
  WARN_TARGETS += warning_cpp17
endif

$(PLUGINS_build): | $(LIBDIR)/libfawkes_skiller_time_estimator.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)

warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting skiller simulator$(TNORMAL) (C++17 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
