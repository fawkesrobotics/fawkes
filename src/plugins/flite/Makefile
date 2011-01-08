#*****************************************************************************
#            Makefile Build System for Fawkes: Flite Plugin
#                            -------------------
#   Created on Tue Oct 28 14:50:37 2008
#   Copyright (C) 2008 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/download.mk

ifeq ($(HAVE_FLITE)$(HAVE_ALSA),11)
  LDFLAGS += $(LDFLAGS_FLITE) $(LDFLAGS_ALSA)
  CFLAGS  += $(CFLAGS_FLITE)  $(CFLAGS_ALSA)
  LIBS_flite = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	       fawkesinterface SpeechSynthInterface flite flite_cmu_us_kal
  OBJS_flite = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp))))
  PLUGINS_all = $(PLUGINDIR)/flite.so
else
  ifneq ($(HAVE_FLITE),1)
    WARN_TARGETS += warning_flite
  endif
  ifneq ($(HAVE_ALSA),1)
    WARN_TARGETS += warning_alsa
  endif
endif

OBJS_all = $(OBJS_flite)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_flite warning_alsa
warning_flite:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting flite plugin$(TNORMAL) (flite[-devel] not installed)"
warning_alsa:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting flite plugin$(TNORMAL) (alsa-lib[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

