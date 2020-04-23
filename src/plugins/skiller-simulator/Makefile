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
                          fawkes_execution_time_estimator \
                          SkillerInterface

OBJS_skiller_simulator  = skiller_simulator_plugin.o exec_thread.o

OBJS_all                = $(OBJS_skiller_simulator)
PLUGINS_all             = $(PLUGINDIR)/skiller-simulator.so

ifeq ($(HAVE_CPP17),1)
  CFLAGS        += $(CFLAGS_CPP17)
  PLUGINS_build  = $(PLUGINS_all)
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: $(WARN_TARGETS)

warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting skiller simulator$(TNORMAL) (C++17 not supported)"
endif

include $(BUILDSYSDIR)/base.mk
