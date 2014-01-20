#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS NavGraph Plugin
#                            -------------------
#   Created on Wed Oct 09 19:23:55 2013
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

CFLAGS += $(CFLAGS_CPP11)

LIBS_clips_navgraph = fawkescore fawkesutils fawkesaspects \
		      fawkesclipsaspect fawkesnavgraphaspect
OBJS_clips_navgraph = clips_navgraph_plugin.o clips_navgraph_thread.o

OBJS_all = $(OBJS_clips_navgraph)

ifeq ($(HAVE_CLIPS),1)
  CFLAGS  += $(CFLAGS_CLIPS)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-navgraph.$(SOEXT)
else
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-navgraph plugin$(TNORMAL) (CLIPS not found)"

endif

include $(BUILDSYSDIR)/base.mk
