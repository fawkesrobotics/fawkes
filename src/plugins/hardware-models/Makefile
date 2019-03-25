#*****************************************************************************
#           Makefile Build System for Fawkes: Hardware Models Plugin
#                            -------------------
#   Created on Sun Mar 24 11:50:46 2019
#   Copyright (C) 2019 by Tim Niemueller
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
include $(BUILDSYSDIR)/clips.mk

LIBS_hardware_models = \
	fawkescore fawkesutils fawkesaspects fawkesblackboard \
        fawkesinterface fawkesclipsaspect
OBJS_hardware_models = hardware_models_plugin.o hardware_models_thread.o

OBJS_all    = $(OBJS_hardware_models)
PLUGINS_all = $(PLUGINDIR)/hardware-models.so

ifeq ($(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_build = $(PLUGINS_all)

	INSTALL_extra = clips_files
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting hardware-models plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
