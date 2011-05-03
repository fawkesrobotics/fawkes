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

PRESUBDIRS = interfaces
SUBDIRS = skelgui

LIBS_openni = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesopenniaspect
OBJS_openni = openni_plugin.o context_thread.o

LIBS_openni_usertracker = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			  fawkesinterface fawkesopenniaspect fawkesopenniutils \
			  fvutils \
			  HumanSkeletonInterface HumanSkeletonProjectionInterface
OBJS_openni_usertracker = usertracker_plugin.o usertracker_thread.o

LIBS_openni_image = fawkescore fawkesutils fawkesaspects  fawkesopenniaspect \
		    fvutils fawkesopenniutils
OBJS_openni_image = image_plugin.o image_thread.o

LIBS_openni_pointcloud = fawkescore fawkesutils fawkesaspects  fawkesopenniaspect \
			 fvutils fawkesopenniutils
OBJS_openni_pointcloud = pointcloud_plugin.o pointcloud_thread.o

LIBS_openni_handtracker = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			  fawkesinterface fawkesopenniaspect fawkesopenniutils \
			  fvutils ObjectPositionInterface
OBJS_openni_handtracker = handtracker_plugin.o handtracker_thread.o

OBJS_all = $(OBJS_openni) $(OBJS_openni_usertracker) $(OBJS_openni_image) \
	   $(OBJS_openni_pointcloud) $(OBJS_openni_handtracker)

ifeq ($(HAVE_OPENNI),1)
  PRESUBDIRS += aspect utils

  CFLAGS  += $(CFLAGS_OPENNI)
  LDFLAGS += $(LDFLAGS_OPENNI)
  PLUGINS_all = $(PLUGINDIR)/openni.so \
		$(PLUGINDIR)/openni-usertracker.so \
		$(PLUGINDIR)/openni-handtracker.so \
		$(PLUGINDIR)/openni-image.so \
		$(PLUGINDIR)/openni-pointcloud.so
else
  WARN_TARGETS += warning_openni
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_openni
warning_openni:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting OpenNI Plugins$(TNORMAL) (openni[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

