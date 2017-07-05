#*****************************************************************************
#        Makefile Build System for Fawkes: Laser PointCloud Plugin
#                            -------------------
#   Created on Tue Nov 15 00:45:57 2011
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

LIBS_laser_pointclouds = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
			 fawkestf fawkespcl_utils fawkesinterface \
			 Laser360Interface Laser720Interface Laser1080Interface

OBJS_laser_pointclouds = laser_pointcloud_plugin.o laser_pointcloud_thread.o

OBJS_all = $(OBJS_laser_pointclouds)

ifeq ($(HAVE_PCL)$(HAVE_TF),11)
  # disable deprecated warnings because glibmm uses deprecated dynamic exception
  # specifications (throw() specifications)
  CFLAGS  += $(CFLAGS_PCL) $(CFLAGS_TF) -Wno-unknown-pragmas -Wno-deprecated
  LDFLAGS += $(LDFLAGS_PCL) $(LDFLAGS_TF)

  PLUGINS_all = $(PLUGINDIR)/laser-pointclouds.so
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_pcl warning_tf
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-pointclouds plugin$(TNORMAL) (PCL not available)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting PCL utility library$(TNORMAL) (tf framework not available)"
endif

include $(BUILDSYSDIR)/base.mk

