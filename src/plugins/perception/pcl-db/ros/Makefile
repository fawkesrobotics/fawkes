#*****************************************************************************
#   Makefile to communicate with ROS for pcl-db-* instructions
#                            -------------------
#   Created on Thu Dec 06 13:28:22 2012
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

BASEDIR = ../../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/ros.mk

LIBS_pcl_db_roscomm = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesrosaspect fawkestf  \
		PclDatabaseMergeInterface PclDatabaseRetrieveInterface
OBJS_pcl_db_roscomm = pcl_db_roscomm_plugin.o pcl_db_roscomm_thread.o

OBJS_all    = $(OBJS_pcl_db_roscomm)

ifeq ($(HAVE_ROS),1)
  ifeq ($(call ros-have-pkg,hybris_c1_msgs),1)
    CFLAGS  += $(CFLAGS_ROS) $(call ros-pkg-cflags,hybris_c1_msgs)
    LDFLAGS += $(LDFLAGS_ROS)

    PLUGINS_all = $(PLUGINDIR)/pcl-db-roscomm.so
  else
    WARN_TARGETS += warning_hybris_c1_msgs
  endif
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_hybris_c1_msgs
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (ROS not found)"
warning_hybris_c1_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (hybris_c1_msgs package not found)"
endif

include $(BUILDSYSDIR)/base.mk

