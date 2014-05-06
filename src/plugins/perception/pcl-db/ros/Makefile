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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_pcl_db_roscomm = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
		fawkesutils fawkeslogging fawkesrosaspect fawkestf fawkespcl_utils \
		PclDatabaseMergeInterface PclDatabaseRetrieveInterface \
		PclDatabaseStoreInterface
OBJS_pcl_db_roscomm = pcl_db_roscomm_plugin.o pcl_db_roscomm_thread.o

OBJS_all    = $(OBJS_pcl_db_roscomm)

REQUIRED_ROS_PKGS = hybris_c1_msgs pcl_conversions

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifeq ($(HAVE_ROS)$(HAVE_PCL)$(HAVE_TF)$(HAVE_BOOST_LIBS),1111)
  ifeq ($(call ros-have-pkgs,$(REQUIRED_ROS_PKGS)),1)
    CFLAGS  += $(CFLAGS_PCL)  $(CFLAGS_ROS)  $(CFLAGS_TF) \
	       $(call ros-pkgs-cflags,$(REQUIRED_ROS_PKGS)) \
	       $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
    LDFLAGS += $(LDFLAGS_PCL) $(LDFLAGS_ROS) $(LDFLAGS_TF) \
	       $(call ros-pkgs-lflags,$(REQUIRED_ROS_PKGS)) \
	       $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

    PLUGINS_all = $(PLUGINDIR)/pcl-db-roscomm.so
  else
    WARN_TARGETS += warning_ros_pkgs
  endif
else
  ifneq ($(HAVE_ROS),1)
    WARN_TARGETS += warning_ros
  endif
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)

.PHONY: warning_ros warning_hybris_c1_msgs
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting pcl-db-roscomm$(TNORMAL) (ROS not found)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting pcl-db-roscomm$(TNORMAL) (PCL not found)"
warning_ros_pkgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting pcl-db-roscomm$(TNORMAL) (missing ROS packages: $(call ros-missing-pkgs,$(REQUIRED_ROS_PKGS)))"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting pcl-db-roscomm$(TNORMAL) (TF framework not available)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build pcl-db-roscomm$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

