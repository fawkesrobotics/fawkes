
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

PRESUBDIRS = aspect interfaces

LIBS_openrave = m fawkescore fawkesaspects fawkesutils fawkesblackboard fawkesopenrave fawkesopenraveaspect fawkesinterface OpenRaveInterface $(LIBS_OPENRAVE)
OBJS_openrave = openrave_plugin.o openrave_thread.o message_handler_thread.o

LIBS_libfawkesopenrave = m fawkescore fawkesutils $(LIBS_OPENRAVE)
OBJS_libfawkesopenrave = environment.o robot.o manipulator.o manipulators/katana6M180.o manipulators/neuronics_katana.o manipulators/kinova_jaco.o

OBJS_all    = $(OBJS_openrave) $(OBJS_libfawkesopenrave)

ifneq ($(HAVE_PYTHON),1)
  WARN_TARGETS += warning_python
endif
ifneq ($(HAVE_OPENRAVE),1)
  WARN_TARGETS += warning_openrave
endif
ifneq ($(HAVE_BOOST_THREAD),1)
  WARN_TARGETS += warning_boost_thread
endif

ifeq ($(HAVE_OPENRAVE)$(HAVE_PYTHON),11)
  CFLAGS  += $(CFLAGS_OPENRAVE) $(CFLAGS_PYTHON)
  LDFLAGS += $(LDFLAGS_OPENRAVE) $(LDFLAGS_PYTHON)
  PLUGINS_all = $(PLUGINDIR)/openrave.so
  LIBS_all = $(LIBDIR)/libfawkesopenrave.so
endif

$(PLUGINDIR)/openrave.$(SOEXT): | $(LIBDIR)/libfawkesopenrave.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openrave warning_python warning_boost_thread
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE Plugin$(TNORMAL) (OpenRAVE not found)"
warning_python:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE Plugin$(TNORMAL) (python not found)"
warning_boost_thread:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE Plugin$(TNORMAL) (Boost thread library not found)"
endif

include $(BUILDSYSDIR)/base.mk
