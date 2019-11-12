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

LIBS_gologpp = fawkescore fawkesutils fawkesblackboard \
               fawkesinterface SkillerInterface
OBJS_gologpp = plugin.o execution_thread.o gologpp_fawkes_backend.o exog_manager.o \
               action_executor.o skiller_action_executor.o \
               aspect/action_executor_dispatcher.o aspect/action_executor_dispatcher_inifin.o \
               utils.o message_action_executor.o sleep_action_executor.o

OBJS_all    = $(OBJS_gologpp)
PLUGINS_all = $(PLUGINDIR)/gologpp.so


ifneq ($(HAVE_CPP17),)
  ifneq ($(PKGCONFIG),)
    HAVE_GOLOGPP = $(if $(shell $(PKGCONFIG) --exists golog++; echo $${?/1/}),1,0)
    ifeq ($(HAVE_GOLOGPP),1)
      CFLAGS += $(shell $(PKGCONFIG) --cflags readylog++ parsegolog++) $(CFLAGS_CPP17)
      LDFLAGS += $(shell $(PKGCONFIG) --libs readylog++ parsegolog++) $(LDFLAGS_CPP17)
      PLUGINS_build = $(PLUGINS_all)
    else
      WARN_TARGETS += warning_gologpp
    endif
  else
    WARN_TARGETS += warning_pkgconfig
  endif
else
  WARN_TARGETS += warning_cpp17
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: $(WARN_TARGETS)
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting golog++ plugin$(TNORMAL) (C++17 not supported)"
warning_pkgconfig:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot search for golog++$(TNORMAL) (pkg-config not found)"
warning_gologpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting golog++ plugin$(TNORMAL) (golog++ libraries not found)"
endif

include $(BUILDSYSDIR)/base.mk

