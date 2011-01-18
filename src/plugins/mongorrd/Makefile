#*****************************************************************************
#         Makefile Build System for Fawkes: MongoDB RRD Plugin
#                            -------------------
#   Created on Sat Jan 15 18:36:57 2011
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
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_mongorrd = fawkescore fawkesutils fawkesaspects fawkesblackboard fawkesinterface \
		fawkesconfig fawkesmongodbaspect fawkesrrdaspect $(LIBS_MONGODB)
OBJS_mongorrd = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_mongorrd)

ifeq ($(HAVE_MONGODB),1)
  PLUGINS_all = $(PLUGINDIR)/mongorrd.so
else
  WARN_TARGETS += warning_mongodb
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Mongo RRD Plugin$(TNORMAL) (mongodb plugin not available)"
endif

include $(BUILDSYSDIR)/base.mk

