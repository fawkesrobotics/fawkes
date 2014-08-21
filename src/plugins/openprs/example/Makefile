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
include $(BUILDSYSDIR)/boost.mk

LIBS_openprs_example = fawkescore fawkesutils fawkesaspects fawkesblackboard \
                       fawkesinterface fawkesopenprsaspect fawkesopenprsutils
OBJS_openprs_example = openprs_example_plugin.o openprs_example_thread.o

OBJS_all = $(OBJS_openprs_example)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

ifneq ($(GCC_TOO_OLD),1)
  ifeq (1,1)
    CFLAGS  += $(CFLAGS_CPP11)

    PLUGINS_all = $(PLUGINDIR)/openprs-example.so
  else
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)  $(WARN_TARGETS_BOOST)

.PHONY: warning_openprs warning_old_gcc $(WARN_TARGETS_BOOST)
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugins$(TNORMAL) ($(OPENPRS_ERROR))"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"
warning_libdaemon:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugins$(TNORMAL) (libdaemon not found)"
$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenPRS Plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

