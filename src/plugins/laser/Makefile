#*****************************************************************************
#            Makefile Build System for Fawkes: Laser Plugin
#                            -------------------
#   Created on Wed Oct 08 13:39:39 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_laser = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	     fawkesinterface Laser360Interface Laser720Interface

ifeq ($(HAVE_LIBPCAN),1)
  LIBS_laser += $(LIBS_LIBPCAN)
  CFLAGS     += $(CFLAGS_LIBPCAN)
else
  FILTER_OUT += lase_edl_aqt.o
  WARN_TARGETS += warning_libpcan
endif

ifeq ($(HAVE_URG),1)
  LIBS_laser += $(LIBS_URG)
  CFLAGS     += $(CFLAGS_URG)
else
  FILTER_OUT += urg_aqt.o
  WARN_TARGETS += warning_urg
endif

ifeq ($(HAVE_URG_GBX),1)
  CFLAGS     += $(CFLAGS_URG_GBX)
  LDFLAGS    += $(LDFLAGS_URG_GBX)
else
  FILTER_OUT += urg_gbx_aqt.o
  WARN_TARGETS += warning_urg_gbx
endif

OBJS_laser = $(filter-out deadspots/% $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
  
ifneq ($(HAVE_LIBPCAN)$(HAVE_URG)$(HAVE_URG_GBX),000)
  PLUGINS_all = $(PLUGINDIR)/laser.so
else
  WARN_TARGETS += warning_nolib
endif

OBJS_all = $(OBJS_laser)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_nolib warning_libpcan warning_urg warning_urg_gbx
warning_nolib:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Laser Plugin$(TNORMAL) (No hardware access library found)"

warning_libpcan:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Lase EDL$(TNORMAL) (libpcan not found)"

warning_urg:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Hokuyo URG$(TNORMAL) (urg library not found)"

warning_urg_gbx:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Hokuyo via Gearbox library$(TNORMAL) (Gearbox library not found)"

endif

include $(BUILDSYSDIR)/base.mk

