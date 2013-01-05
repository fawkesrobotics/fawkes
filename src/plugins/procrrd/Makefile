#*****************************************************************************
#              Makefile Build System for Fawkes: ProcRRD Plugin
#                            -------------------
#   Created on Tue Dec 18 9:57:23 2012
#   Copyright (C) 2012 by Bastian Klingen
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
include $(LIBSRCDIR)/utils/utils.mk

LIBS_procrrd = fawkescore fawkesutils fawkesaspects fawkesconfig fawkesrrdaspect
OBJS_procrrd = procrrd_plugin.o procrrd_thread.o

OBJS_all    = $(OBJS_sysrrd)
PLUGINS_all = $(PLUGINDIR)/procrrd.$(SOEXT)

ifneq ($(OS),Linux)
  WARN_TARGETS += warning_onlylinux
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_onlylinux
warning_onlylinux:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ProcRRD Plugin$(TNORMAL) (currently only Linux is supported)"
endif

include $(BUILDSYSDIR)/base.mk

