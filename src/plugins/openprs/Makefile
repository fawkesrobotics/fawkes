#*****************************************************************************
#               Makefile Build System for Fawkes: OpenPRS Plugin
#                            -------------------
#   Created on Thu Aug 14 15:53:07 2014
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

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/openprs.mk

LIBS_openprs = fawkescore fawkesutils fawkesaspects fawkesblackboard \
               fawkesinterface fawkesopenprsaspect fawkesopenprsutils
OBJS_openprs = openprs_plugin.o openprs_thread.o

OBJS_all = $(OBJS_openprs)

ifeq ($(HAVE_OPENPRS),1)
  PRESUBDIRS += utils aspect
  SUBDIRS    += example

  CFLAGS  += $(CFLAGS_OPENPRS)  $(CFLAGS_OPENPRS_MP) 
  LDFLAGS += $(LDFLAGS_OPENPRS) $(LDFLAGS_OPENPRS_MP)

  PLUGINS_all = $(PLUGINDIR)/openprs.so
else
  WARN_TARGETS += warning_openprs
endif

aspect: utils
example: aspect utils

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openprs
warning_openprs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Aspects$(TNORMAL) ($(OPENPRS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk

