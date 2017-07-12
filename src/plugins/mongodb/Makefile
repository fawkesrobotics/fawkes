#*****************************************************************************
#              Makefile Build System for Fawkes: MongoDB Plugin
#                            -------------------
#   Created on Sun Dec 05 23:15:27 2010 (Steelers vs. Baltimore)
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
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

INCDIRS += $(INCDIRS_MONGODB)

LIBS_mongodb = fawkescore fawkesaspects fawkesmongodbaspect
OBJS_mongodb = mongodb_plugin.o mongodb_thread.o

LIBS_mongodb_rrd = fawkescore fawkesutils fawkesaspects fawkesblackboard fawkesinterface \
                fawkesconfig fawkesmongodbaspect fawkesrrdaspect
OBJS_mongodb_rrd = mongorrd_plugin.o mongorrd_thread.o


CFLAGS  += $(CFLAGS_MONGODB) $(CFLAGS_CPP14)
LDFLAGS += $(LDFLAGS_MONGODB)

OBJS_all    = $(OBJS_mongodb) $(OBJS_mongodb_rrd)

ifeq ($(HAVE_MONGODB)$(HAVE_CPP14),11)
  PLUGINS_all = $(PLUGINDIR)/mongodb.$(SOEXT) $(PLUGINDIR)/mongodb-rrd.$(SOEXT)
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_CPP14),1)
    WARN_TARGETS += warning_cpp14
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_cpp14
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB Plugins$(TNORMAL) (mongodb[-devel] not installed)"
warning_cpp14:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB Plugins$(TNORMAL) (C++ compiler too old, require at least C++14)"
endif

include $(BUILDSYSDIR)/base.mk

