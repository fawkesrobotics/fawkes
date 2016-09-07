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
include $(BUILDCONFDIR)/navgraph/navgraph.mk

LIBS_clips_navgraph = fawkescore fawkesutils fawkesaspects \
		      fawkesclipsaspect fawkesnavgraphaspect \
		      fawkesnavgraph
OBJS_clips_navgraph = clips_navgraph_plugin.o clips_navgraph_thread.o

OBJS_all = $(OBJS_clips_navgraph)

# Extra files for system-wide install, i.e., "make install"
FILES_clips_files   = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/clips/*.clp))
DESTDIR_clips_files = $(EXEC_SHAREDIR)/clips/clips-navgraph

ifeq ($(HAVE_CPP11)$(HAVE_CLIPS)$(HAVE_NAVGRAPH),111)
  CFLAGS  += $(CFLAGS_CLIPS) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_CLIPS)

  PLUGINS_all = $(PLUGINDIR)/clips-navgraph.$(SOEXT)

	INSTALL_extra = clips_files
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS = warning_clips
  endif
  ifneq ($(HAVE_NAVGRAPH),1)
    WARN_TARGETS = warning_navgraph
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_clips warning_cpp11
warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting clips-navgraph plugin$(TNORMAL) ($(CLIPS_ERROR))"

warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-navgraph plugin$(TNORMAL) (C++11 not supported)"

warning_navgraph:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-navgraph plugin$(TNORMAL) (Navgraph not available)"

endif

include $(BUILDSYSDIR)/base.mk
