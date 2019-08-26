#*****************************************************************************
#               Makefile Build System for Fawkes: golog++ plugin
#                            -------------------
#   Copyright (C) 2019, Victor Mataré
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

LIBS_clips = fawkescore fawkesutils fawkesblackboard \
           fawkesinterface 
OBJS_clips = plugin.o execution_thread.o gologpp_fawkes_backend.o blackboard.o

OBJS_all    = $(OBJS_clips)
PLUGINS_all = $(PLUGINDIR)/gologpp.so

ifneq ($(PKGCONFIG),)
  HAVE_GOLOGPP = $(if $(shell $(PKGCONFIG) --exists gologpp; echo $${?/1/}),1,0)
  ifeq ($(HAVE_GOLOGPP),1)
    CFLAGS += $(shell $(PKGCONFIG) --cflags readylogpp)
    LDFLAGS += $(shell $(PKGCONFIG) --libs readylogpp)
  else
    WARN_TARGETS += warning_gologpp
  endif
else
  WARN_TARGETS += warning_pkgconfig
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_pkgconfig warning_gologpp
warning_pkgconfig:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot search for golog++$(TNORMAL) (pkg-config not found)"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting golog++ plugin$(TNORMAL) (golog++ libraries not found)"
endif

include $(BUILDSYSDIR)/base.mk

