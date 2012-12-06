#*****************************************************************************
#   Makefile for Fawkes plugin to restore and merge point clouds from MongoDB
#                            -------------------
#   Created on Wed Nov 28 10:50:24 2012
#   Copyright (C) 2012 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

PRESUBDIRS = interfaces

# Add -DUSE_TIMETRACKER to enable time tracking
CFLAGS += -DUSE_TIMETRACKER

LIBS_pcl_db_merge = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesmongodbaspect fawkestf fawkespcl_utils \
		PclDatabaseMergeInterface
OBJS_pcl_db_merge = pcl_db_merge_plugin.o pcl_db_merge_thread.o mongodb_tf_transformer.o

OBJS_all    = $(OBJS_pcl_db_merge)

REQUIRED_PCL_LIBS = sample_consensus segmentation filters surface registration

ifeq ($(HAVE_MONGODB)$(HAVE_PCL)$(HAVE_TF),111)
  ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
    CFLAGS += $(CFLAGS_PCL) $(CFLAGS_TF) $(CFLAGS_MONGODB) \
	      $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS))
    LDFLAGS += $(LDFLAGS_PCL) $(LDFLAGS_TF) $(LDFLAGS_MONGODB) \
	       $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

    PLUGINS_all = $(PLUGINDIR)/pcl-db-merge.so
  else
    WARN_TARGETS += warning_pcl_components
  endif
else
  ifeq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifeq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifeq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_pcl warning_pcl_components warning_tf
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (mongodb[-devel] not installed)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (pcl[-devel] not installed)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (TF framework not available)"
endif

include $(BUILDSYSDIR)/base.mk

