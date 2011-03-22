#*****************************************************************************
#              Makefile Build System for Fawkes: ECLiPSe externals
#                            -------------------
#   Created on Wed Mar 09 17:34:04 2011
#   copyright (C) 2011 by Daniel Beck, AllemaniACs RoboCup Team
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
include $(BASEDIR)/src/libs/eclipse_externals/eclipse.mk

CFLAGS += $(ECLIPSE_CFLAGS)
LDFLAGS += $(ECLIPSE_LDFLAGS)

LIBS_eclipse_externals = fawkescore fawkesblackboard fawkesinterface fawkesutils eclipse
OBJS_eclipse_externals = blackboard.o rcsoft_map_graph.o

OBJS_all = $(OBJS_eclipse_externals)

ifeq ($(HAVE_ECLIPSE),1)
  LIBS_all = $(LIBDIR)/eclipse_externals.so
else
  WARN_TARGETS += warning_eclipse
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_eclipse
warning_eclipse:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ECLiPSe externals(TNORMAL) ($(ECLIPSE_FAIL_REASON))"
endif

include $(BUILDSYSDIR)/base.mk
