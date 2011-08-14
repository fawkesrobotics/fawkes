#*****************************************************************************
#            Makefile Build System for Fawkes: Katana Plugin
#                            -------------------
#   Created on Mon Jun 08 17:54:53 2009
#   Copyright (C) 2006-2009 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_katana = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface KatanaInterface
OBJS_katana = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

#CFLAGS += -DUSE_TIMETRACKER

ifneq ($(PKGCONFIG),)
  HAVE_LIBKNI := $(if $(shell $(PKGCONFIG) --exists 'libkni3'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBKNI),1)
  CFLAGS_LIBKNI  = -DHAVE_KNI $(shell $(PKGCONFIG) --cflags 'libkni3')
  LDFLAGS_LIBKNI = $(shell $(PKGCONFIG) --libs 'libkni3')
endif
ifneq ($(PKGCONFIG),)
  HAVE_OPENRAVE := $(if $(shell $(PKGCONFIG) --exists 'openrave'; echo $${?/1/}) \
                     OR $(shell $(PKGCONFIG) --exists 'openrave-0.3'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_OPENRAVE),1)
  CFLAGS_OPENRAVE    = $(shell openrave-config --cflags-only-I) -DHAVE_OPENRAVE
  LDFLAGS_OPENRAVE   = $(shell openrave-config --libs-core)
endif

ifeq ($(HAVE_LIBKNI),1)
  CFLAGS  += $(CFLAGS_LIBKNI) -Wno-error
  LDFLAGS += $(LDFLAGS_LIBKNI)
  PLUGINS_all = $(PLUGINDIR)/katana.so
  ifeq ($(HAVE_OPENRAVE),1)
    LIBS_katana += fawkesopenrave fawkesopenraveaspect
    CFLAGS  += $(CFLAGS_OPENRAVE)
    LDFLAGS += $(LDFLAGS_OPENRAVE)
  else
    WARN_TARGETS += warning_openrave
  endif
else
  WARN_TARGETS += warning_kni
endif

OBJS_all = $(OBJS_katana)

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BUILDSYSDIR)/interface.mk

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_kni
warning_kni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Katana Plugin$(TNORMAL) (KNI[-devel] not installed)"

.PHONY: warning_openrave
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE support for Katana Plugin$(TNORMAL) (OpenRAVE not installed)"
endif

include $(BUILDSYSDIR)/base.mk

