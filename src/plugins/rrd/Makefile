#*****************************************************************************
#              Makefile Build System for Fawkes: RRD Plugin
#                            -------------------
#   Created on Sun Dec 05 23:15:27 2010 (Steelers vs. Baltimore)
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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

PRESUBDIRS = aspect

CFLAGS  += $(CFLAGS_RRD)
LDFLAGS += $(LDFLAGS_RRD)

LIBS_rrd = m fawkescore fawkesaspects fawkesutils fawkesrrdaspect $(LIBS_RRD)
OBJS_rrd = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_rrd)

ifeq ($(HAVE_RRD),1)
  PLUGINS_all = $(PLUGINDIR)/rrd.so
else
  WARN_TARGETS += warning_rrd
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_rrd
warning_rrd:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RRD Plugin$(TNORMAL) (rrdtool[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
