#*****************************************************************************
#            Makefile Build System for Fawkes: OpenNI Plugin
#                            -------------------
#   Created on Thu Feb 17 10:17:34 2011
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk


PRESUBDIRS = interfaces utils
SUBDIRS = skelgui pclviewer

LIBS_openni = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesopenniaspect
OBJS_openni = openni_plugin.o context_thread.o

LIBS_openni_usertracker = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			  fawkesinterface fawkesopenniaspect fawkesopenni_utils \
			  fvutils \
			  HumanSkeletonInterface HumanSkeletonProjectionInterface
OBJS_openni_usertracker = usertracker_plugin.o usertracker_thread.o

LIBS_openni_data = fawkescore fawkesutils fawkesaspects  fawkesopenniaspect \
		    fvutils fawkesopenni_utils fawkestf
OBJS_openni_data = data_plugin.o image_thread.o depth_thread.o pointcloud_thread.o

LIBS_openni_pcl_frombuf = fawkescore fawkesutils fawkesaspects  fawkesopenniaspect \
			 fvutils fawkesopenni_utils
OBJS_openni_pcl_frombuf = pcl_frombuf_plugin.o pcl_frombuf_thread.o

LIBS_openni_handtracker = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			  fawkesinterface fawkesopenniaspect fawkesopenni_utils \
			  fvutils ObjectPositionInterface
OBJS_openni_handtracker = handtracker_plugin.o handtracker_thread.o

ifeq ($(HAVE_PCL)$(HAVE_TF),11)
  CFLAGS_pointcloud_thread  = $(CFLAGS) $(CFLAGS_PCL) $(CFLAGS_TF)
  CFLAGS_data_plugin        = $(CFLAGS) $(CFLAGS_PCL) $(CFLAGS_TF)
  LDFLAGS_openni_data       = $(LDFLAGS_PCL) $(LDFLAGS_TF)
  LIBS_openni_data         += fvutils fawkestf fawkespcl_utils

  CFLAGS_pcl_frombuf_thread = $(CFLAGS) $(CFLAGS_PCL) $(CFLAGS_TF)
  CFLAGS_pcl_frombuf_plugin = $(CFLAGS) $(CFLAGS_PCL) $(CFLAGS_TF)
  LDFLAGS_openni_pcl_frombuf = $(LDFLAGS_PCL) $(LDFLAGS_TF)
  LIBS_openni_pcl_frombuf += fvutils fawkestf fawkespcl_utils
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

OBJS_all = $(OBJS_openni) $(OBJS_openni_usertracker) $(OBJS_openni_data) \
	   $(OBJS_openni_handtracker) $(OBJS_openni_pcl_frombuf)

# Ordering constraints
utils: interfaces
skelgui: utils
pclviewer: utils


ifeq ($(HAVE_OPENNI),1)
  PRESUBDIRS += aspect

  CFLAGS  += $(CFLAGS_OPENNI) -Wno-deprecated-declarations
  LDFLAGS += $(LDFLAGS_OPENNI)
  PLUGINS_all = $(PLUGINDIR)/openni.$(SOEXT) \
		$(PLUGINDIR)/openni-usertracker.$(SOEXT) \
		$(PLUGINDIR)/openni-handtracker.$(SOEXT) \
		$(PLUGINDIR)/openni-data.$(SOEXT)

  ifeq ($(HAVE_PCL),1)
    PLUGINS_all += $(PLUGINDIR)/openni-pcl-frombuf.$(SOEXT)
  endif
else
  WARN_TARGETS += warning_openni
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openni warning_pcl warning_tf
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenNI Plugins$(TNORMAL) (openni[-devel] not installed)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL part in openni-pointcloud and openni-pcl-frombuf$(TNORMAL) (pcl[-devel] not installed)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (tf framework not available)"
endif

include $(BUILDSYSDIR)/base.mk

