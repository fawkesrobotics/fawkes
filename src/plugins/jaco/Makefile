
#*****************************************************************************
#              Makefile Build System for Fawkes: Kinova Jaco Plugin
#                            -------------------
#   Created on Tue Jun 04 13:13:20 2013
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
include $(BASEDIR)/src/plugins/openrave/openrave.mk

PRESUBDIRS = interfaces

LIBS_jaco = fawkescore fawkesaspects fawkesblackboard fawkesutils fawkesinterface JacoInterface JacoBimanualInterface m pthread
OBJS_jaco = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

ifeq ($(HAVE_LIBKINDRV),1)
  CFLAGS  += $(CFLAGS_LIBKINDRV)
  LDFLAGS += $(LDFLAGS_LIBKINDRV)
  PLUGINS_all = $(PLUGINDIR)/jaco.so
  ifeq ($(HAVE_OPENRAVE)$(HAVE_PYTHON),11)
    LIBS_jaco += fawkesopenrave fawkesopenraveaspect
    CFLAGS += $(CFLAGS_OPENRAVE)
    LDFLAGS += $(LDFLAGS_OPENRAVE)
  else
    WARN_TARGETS += warning_openrave
  endif
else
  WARN_TARGETS += warning_libkindrv
endif

OBJS_all    = $(OBJS_jaco)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_libkindrv
warning_libkindrv:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Jaco Plugin$(TNORMAL) (libkindrv not found)"

.PHONY: warning_openrave
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE support for Jaco Plugin$(TNORMAL) (openrave plugin not available)"
endif

include $(BUILDSYSDIR)/base.mk
