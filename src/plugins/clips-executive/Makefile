#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS Executive Plugin
#                            -------------------
#   Created on Tue Sep 19 11:50:46 2017
#   Copyright (C) 2006-2017 by Tim Niemueller
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

SUBDIRS += rest-api

LIBS_clips_executive = \
	fawkescore fawkesutils fawkesaspects fawkesblackboard \
        fawkesinterface fawkesclipsaspect
OBJS_clips_executive = clips_executive_plugin.o clips_executive_thread.o map_skill.o

OBJS_all = $(OBJS_clips_executive)

# Extra files for system-wide install, i.e., "make install"
FILES_clips_files   = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/clips/*.clp))
DESTDIR_clips_files = $(EXEC_SHAREDIR)/clips/clips-executive

ifeq ($(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-executive.so

	INSTALL_extra = clips_files
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-executive plugin$(TNORMAL) ($(CLIPS_ERROR))"
endif

include $(BUILDSYSDIR)/base.mk
