#*****************************************************************************
#            Makefile Build System for Fawkes: OpenNI Plugin
#                            -------------------
#   Created on Thu Feb 17 10:17:34 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_openni = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface
OBJS_openni = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

#CFLAGS += -DUSE_TIMETRACKER

ifneq ($(wildcard $(SYSROOT)/usr/include/ni/XnContext.h),)
  HAVE_OPENNI = 1
endif
ifeq ($(HAVE_OPENNI),1)
  CFLAGS  = -DHAVE_OPENNI
  PLUGINS_all = $(PLUGINDIR)/openni.so
else
  WARN_TARGETS += warning_openni
endif

OBJS_all = $(OBJS_openni)

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BUILDSYSDIR)/interface.mk

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openni
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenNI Plugin$(TNORMAL) (openni[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

