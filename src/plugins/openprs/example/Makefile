#*****************************************************************************
#          Makefile Build System for Fawkes: OpenPRS Example Plugin
#                            -------------------
#   Created on Tue Aug 19 11:47:27 2014
#   Copyright (C) 2014 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/openprs.mk

LIBS_openprs_example = fawkescore fawkesutils fawkesaspects fawkesblackboard \
                       fawkesinterface fawkesopenprsaspect fawkesopenprsutils
OBJS_openprs_example = openprs_example_plugin.o openprs_example_thread.o

OBJS_all = $(OBJS_openprs_example)

ifeq ($(HAVE_OPENPRS),1)
  CFLAGS  += $(CFLAGS_OPENPRS)
  LDFLAGS += $(LDFLAGS_OPENPRS)

  PLUGINS_all = $(PLUGINDIR)/openprs-example.so
else
  WARN_TARGETS += warning_openprs
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openprs
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Aspects$(TNORMAL) ($(OPENPRS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk

