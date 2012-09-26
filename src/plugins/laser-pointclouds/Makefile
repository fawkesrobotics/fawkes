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

LIBS_laser_pointclouds = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
			 fawkesinterface Laser360Interface Laser720Interface \

OBJS_laser_pointclouds = laser_pointcloud_plugin.o laser_pointcloud_thread.o

OBJS_all = $(OBJS_laser_pointclouds)

ifeq ($(HAVE_PCL),1)
  CFLAGS  += $(CFLAGS_PCL) -Wno-unknown-pragmas -Wno-deprecated-declarations
  LDFLAGS += $(LDFLAGS_PCL)

  PLUGINS_all = $(PLUGINDIR)/laser-pointclouds.so
else
  WARN_TARGETS += warning_pcl
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_pcl
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser-pointclouds plugin$(TNORMAL) (PCL not available)"
endif

include $(BUILDSYSDIR)/base.mk

