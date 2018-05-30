#*****************************************************************************
#        Makefile Build System for Fawkes: Laser Filter Plugin
#                            -------------------
#   Created on Sun Mar 13 01:05:36 2011
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
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces
SUBDIRS = deadspots

LIBS_laser_filter = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	            fawkesinterface \
	            fawkes_amcl_utils fawkes_amcl_map \
		    Laser360Interface Laser720Interface Laser1080Interface \
				LaserBoxFilterInterface

ifeq ($(HAVE_TF),1)
  CFLAGS  += $(CFLAGS_TF) -Wno-deprecated-declarations
  LDFLAGS += $(LDFLAGS_TF)
  LIBS_laser_filter += fawkestf
else
  FILTER_OUT += filters/projection.o
  WARN_TARGETS = warning_tf
endif

OBJS_laser_filter = $(filter-out deadspots/% $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp)))))
OBJS_all = $(OBJS_laser_filter)

PLUGINS_all = $(PLUGINDIR)/laser-filter.so

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting projection filter$(TNORMAL) (tf not available)"
endif

include $(BUILDSYSDIR)/base.mk

