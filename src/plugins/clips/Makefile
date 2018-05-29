#*****************************************************************************
#               Makefile Build System for Fawkes: CLIPS Plugin
#                            -------------------
#   Created on Sat Jun 16 14:42:34 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_clips = fawkescore fawkesutils fawkesaspects fawkesblackboard \
           fawkesinterface fawkesclipsaspect
OBJS_clips = clips_plugin.o clips_thread.o \
	     feature_blackboard.o feature_config.o feature_redefine_warning.o

OBJS_all = $(OBJS_clips)

# Extra files for system-wide install, i.e., "make install"
FILES_clips_files   = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/clips/*.clp))
DESTDIR_clips_files = $(EXEC_SHAREDIR)/clips/clips

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,6),1)
    GCC_TOO_OLD=1
  endif
endif

ifneq ($(GCC_TOO_OLD),1)
  ifeq ($(HAVE_CLIPS),1)
    PRESUBDIRS += aspect
		SUBDIRS += rest-api

    CFLAGS  += $(CFLAGS_CLIPS)
    LDFLAGS += $(LDFLAGS_CLIPS)

    PLUGINS_all = $(PLUGINDIR)/clips.so

		INSTALL_extra = clips_files
else
    WARN_TARGETS += warning_clips
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_old_gcc
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Plugins$(TNORMAL) ($(CLIPS_ERROR))"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS Plugins$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.6)"
endif

include $(BUILDSYSDIR)/base.mk

