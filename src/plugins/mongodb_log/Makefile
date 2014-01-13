#*****************************************************************************
#           Makefile Build System for Fawkes: MongoDB Logging Plugin
#                            -------------------
#   Created on Wed Dec 08 22:52:40 2010
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

SUBDIRS=tools

LIBS_mongodb_log = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesmongodbaspect fvutils \
		fawkestf fawkespcl_utils
OBJS_mongodb_log = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

CFLAGS  += $(CFLAGS_MONGODB)
LDFLAGS += $(LDFLAGS_MONGODB)

OBJS_all    = $(OBJS_mongodb_log)

ifeq ($(HAVE_MONGODB)$(HAVE_PCL)$(HAVE_TF),111)
  CFLAGS += $(CFLAGS_PCL) $(CFLAGS_TF) $(CFLAGS_MONGODB)
  LDFLAGS += $(LDFLAGS_PCL) $(LDFLAGS_TF) $(LDFLAGS_MONGODB)
  PLUGINS_all = $(PLUGINDIR)/mongodb-log.so
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_pcl warning_tf
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB Logging Plugin$(TNORMAL) (mongodb[-devel] not installed)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (pcl[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (tf framework not available)"
endif

include $(BUILDSYSDIR)/base.mk

