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

ifeq ($(HAVE_CPP11)$(HAVE_CLIPS),11)
  PRESUBDIRS += aspect

  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips.so
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_cpp11
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CLIPS plugin$(TNORMAL) (CLIPS not found)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build CLIPS plugin$(TNORMAL) (C++11 not supported)"
endif

include $(BUILDSYSDIR)/base.mk

