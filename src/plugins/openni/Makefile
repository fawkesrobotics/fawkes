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
		    fvutils fawkesopenni_utils
OBJS_openni_data = data_plugin.o image_thread.o depth_thread.o pointcloud_thread.o

LIBS_openni_pcl_frombuf = fawkescore fawkesutils fawkesaspects  fawkesopenniaspect \
			 fvutils fawkesopenni_utils
OBJS_openni_pcl_frombuf = pcl_frombuf_plugin.o pcl_frombuf_thread.o

LIBS_openni_handtracker = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			  fawkesinterface fawkesopenniaspect fawkesopenni_utils \
			  fvutils ObjectPositionInterface
OBJS_openni_handtracker = handtracker_plugin.o handtracker_thread.o

ifeq ($(HAVE_PCL),1)
  CFLAGS_pointcloud_thread  = $(CFLAGS) $(CFLAGS_PCL)
  CFLAGS_data_plugin        = $(CFLAGS) $(CFLAGS_PCL)
  LDFLAGS_openni_data       = $(LDFLAGS_PCL)
  LIBS_openni_data         += fvutils

  CFLAGS_pcl_frombuf_thread = $(CFLAGS) $(CFLAGS_PCL)
  CFLAGS_pcl_frombuf_plugin = $(CFLAGS) $(CFLAGS_PCL)
  LDFLAGS_openni_pcl_frombuf = $(LDFLAGS_PCL)
  LIBS_openni_pcl_frombuf += fvutils
else
  WARN_TARGETS += warning_pcl
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

.PHONY: warning_openni warning_pcl
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenNI Plugins$(TNORMAL) (openni[-devel] not installed)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL part in openni-pointcloud and openni-pcl-frombuf$(TNORMAL) (pcl[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

