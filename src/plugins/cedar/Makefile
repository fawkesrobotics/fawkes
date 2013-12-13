#*****************************************************************************
#           Makefile Build System for Fawkes: CEDAR Plugin
#                            -------------------
#   Created on Fri Aug 16 17:42:19 2013
#   Copyright (C) 2006-2013 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_cedar = \
	fawkescore fawkesutils fawkesaspects fawkesplugin fawkesblackboard \
        fawkesinterface fawkesclipsaspect
OBJS_cedar = cedar_plugin.o cedar_thread.o plugin_director_thread.o

OBJS_all = $(OBJS_cedar)

ifeq ($(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/cedar.$(SOEXT)
else
  WARN_TARGETS += warning_clips
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting CEDAR plugin$(TNORMAL) (CLIPS not found)"
endif

include $(BUILDSYSDIR)/base.mk
