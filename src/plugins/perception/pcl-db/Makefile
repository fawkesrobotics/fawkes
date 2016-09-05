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

# Add -DUSE_TIMETRACKER to enable time tracking
CFLAGS += -DUSE_TIMETRACKER

PRESUBDIRS = interfaces

ifeq ($(PCL_USES_OPENMP),1)
  ifneq ($(USE_OPENMP),1)
    CFLAGS  += $(CFLAGS_OPENMP)
    LDFLAGS += $(LDFLAGS_OPENMP)
  endif
endif

LIBS_pcl_db_merge = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesmongodbaspect fawkestf fawkespcl_utils \
		PclDatabaseMergeInterface
OBJS_pcl_db_merge = pcl_db_merge_plugin.o pcl_db_merge_thread.o mongodb_tf_transformer.o

REQUIRED_PCL_LIBS_MERGE = sample_consensus segmentation filters surface registration


LIBS_pcl_db_retrieve = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		   fawkesutils fawkeslogging fawkesmongodbaspect fawkestf fawkespcl_utils \
		   PclDatabaseRetrieveInterface
OBJS_pcl_db_retrieve = pcl_db_retrieve_plugin.o pcl_db_retrieve_thread.o \
			mongodb_tf_transformer.o

LIBS_pcl_db_store = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		   fawkesutils fawkeslogging fawkesmongodbaspect fawkespcl_utils \
		   PclDatabaseStoreInterface
OBJS_pcl_db_store = pcl_db_store_plugin.o pcl_db_store_thread.o

OBJS_all    = $(OBJS_pcl_db_merge) $(OBJS_pcl_db_retrieve) $(OBJS_pcl_db_store)


ifeq ($(HAVE_MONGODB)$(HAVE_PCL)$(HAVE_TF)$(HAVE_CPP11),1111)
  CFLAGS += $(CFLAGS_PCL) $(CFLAGS_TF) $(CFLAGS_MONGODB) $(CFLAGS_CPP11)
  LDFLAGS += $(LDFLAGS_PCL) $(LDFLAGS_TF) $(LDFLAGS_MONGODB)

  PLUGINS_all = $(PLUGINDIR)/pcl-db-store.so $(PLUGINDIR)/pcl-db-retrieve.so

  SUBDIRS = ros

  ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS_MERGE)),1)
    CFLAGS += $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS_MERGE)) \
	      -Wno-sign-compare
    LDFLAGS_pcl_db_merge +=  $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS_MERGE))

    PLUGINS_all += $(PLUGINDIR)/pcl-db-merge.so
  else
    WARN_TARGETS += warning_pcl_components
  endif
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
endif

ros: interfaces


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_pcl warning_pcl_components warning_tf warning_cpp11
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL plugins$(TNORMAL) (mongodb[-devel] not installed)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL plugins$(TNORMAL) (pcl[-devel] not installed)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS_MERGE)))"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (TF framework not available)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB PCL merge plugin$(TNORMAL) (C++11 support required)"
endif

include $(BUILDSYSDIR)/base.mk

