#*****************************************************************************
#              Makefile Build System for Fawkes: OpenRAVE Plugin
#                            -------------------
#   Created on Fri Feb 25 15:08:00 2011
#   Copyright (C) 2011 by Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

CFLAGS  += $(CFLAGS_OPENRAVE)
LDFLAGS += $(LDFLAGS_OPENRAVE)

LIBS_openrave = m fawkescore fawkesaspects fawkesutils fawkesopenraveaspect $(LIBS_OPENRAVE)
OBJS_openrave = $(patsubst %.cpp,%.o,$(filter-out $(OBJS_FILTER_OUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp))))))

OBJS_all    = $(OBJS_openrave)

ifeq ($(HAVE_OPENRAVE),1)
  PLUGINS_all = $(PLUGINDIR)/openrave.so
else
  WARN_TARGETS += warning_openrave
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openrave
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE Plugin$(TNORMAL) (OpenRAVE not installed)"
endif

include $(BUILDSYSDIR)/base.mk
