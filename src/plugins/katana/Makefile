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
include $(BASEDIR)/src/plugins/openrave/openrave.mk
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces

LIBS_katana = fawkescore fawkesutils fawkesaspects fawkesblackboard fawkestf \
	      fawkesinterface KatanaInterface JointInterface
OBJS_katana = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))

#CFLAGS += -DUSE_TIMETRACKER

ifneq ($(PKGCONFIG),)
  HAVE_LIBKNI := $(if $(shell $(PKGCONFIG) --exists 'libkni3'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBKNI),1)
  CFLAGS_LIBKNI  = -DHAVE_KNI $(shell $(PKGCONFIG) --cflags 'libkni3')
  LDFLAGS_LIBKNI = $(shell $(PKGCONFIG) --libs 'libkni3')
else
  WARN_TARGETS += warning_kni
endif

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(HAVE_LIBKNI)$(HAVE_TF),11)
  # Set flag -DEARLY_PLANNING if you want to run the path-planning part outside of
  # the motion-thread. This is a temporary solution until the source of yet unresolved error
  # in Qt/Coin/OpenRAVE is found and fixed. This behaviour is NOT preferred, as the planning
  # will then be executed in a non-continuous thread!
  CFLAGS  += $(CFLAGS_LIBKNI) -DEARLY_PLANNING
  LDFLAGS += $(LDFLAGS_LIBKNI)
  # KNI uses std::auto_ptr which is deprecated in C++11
  # Ignore this warning here as we won't fix KNI for this
  CFLAGS_controller_kni = $(CFLAGS) -Wno-deprecated-declarations
  PLUGINS_all = $(PLUGINDIR)/katana.so
  ifeq ($(HAVE_OPENRAVE)$(HAVE_PYTHON),11)
    LIBS_katana += fawkesopenrave fawkesopenraveaspect
    CFLAGS  += $(CFLAGS_OPENRAVE)
    LDFLAGS += $(LDFLAGS_OPENRAVE)
  else
    WARN_TARGETS += warning_openrave
  endif
endif

OBJS_all = $(OBJS_katana)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Katana Plugin$(TNORMAL) (fawkestf not available)"

.PHONY: warning_kni
warning_kni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Katana Plugin$(TNORMAL) (KNI[-devel] not installed)"

.PHONY: warning_openrave
warning_openrave:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenRAVE support for Katana Plugin$(TNORMAL) (openrave plugin not available)"
endif

include $(BUILDSYSDIR)/base.mk

