
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

CFLAGS  += $(CFLAGS_OPENRAVE) -Wno-error
LDFLAGS += $(LDFLAGS_OPENRAVE)

LIBS_openrave = m fawkescore fawkesaspects fawkesutils fawkesblackboard fawkesopenrave fawkesopenraveaspect fawkesinterface OpenRaveInterface $(LIBS_OPENRAVE)
OBJS_openrave = or_plugin.o or_thread.o or_message_handler_thread.o

LIBS_libfawkesopenrave = m fawkescore fawkesutils $(LIBS_OPENRAVE)
OBJS_libfawkesopenrave = environment.o robot.o manipulator.o manipulators/katana6M180.o manipulators/neuronics_katana.o

OBJS_all    = $(OBJS_openrave) $(OBJS_libfawkesopenrave)

ifeq ($(HAVE_OPENRAVE),1)
  PLUGINS_all = $(PLUGINDIR)/openrave.so
  LIBS_all = $(LIBDIR)/libfawkesopenrave.so
else
  WARN_TARGETS += warning_openrave
endif

$(PLUGINDIR)/openrave.$(SOEXT): | $(LIBDIR)/libfawkesopenrave.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

INTERFACES_all = $(notdir $(patsubst %.xml,%,$(wildcard $(SRCDIR)/*.xml)))
include $(BUILDSYSDIR)/interface.mk

.PHONY: warning_openrave
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE Plugin$(TNORMAL) (OpenRAVE not installed)"
endif

include $(BUILDSYSDIR)/base.mk
