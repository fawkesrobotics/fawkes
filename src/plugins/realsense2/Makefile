#*****************************************************************************
#         Makefile Build System for Fawkes: realsense Plugin
#                            -------------------
#   Created on Mon Jun 13 17:09:44 2016
#   Copyright (C) 2016 by Johannes Rothe
#                 2018 by Till Hofmann
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

CFLAGS += $(CFLAGS_CPP11)
CFLAGS  += $(CFLAGS_TF) $(CFLAGS_PCL) $(CFFLAGS_conveyor_pose)
LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL)

LIBS_realsense2 = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface \
                      fawkespcl_utils SwitchInterface

OBJS_realsense2 = realsense2_plugin.o realsense2_thread.o 

OBJS_all    = $(OBJS_realsense2)
PLUGINS_all = $(PLUGINDIR)/realsense2.$(SOEXT)

REALSENSE2_INCLUDE_PATHS_ = /usr/include/librealsense2 /usr/local/include/librealsense2
REALSENSE_INCLUDE_PATHS_ = /usr/include/librealsense /usr/local/include/librealsense

ifneq ($(wildcard $(addsuffix /rs.h,$(REALSENSE2_INCLUDE_PATHS_))),)
  # The old librealsense has been moved to librealsense1, librealsense is now
  # librealsense2.
  CFLAGS += -DHAVE_REALSENSE2
  LIBS_realsense2 += realsense2
  PLUGINS_build = $(PLUGINS_all)
else ifneq ($(wildcard $(addsuffix /rs.h,$(REALSENSE_INCLUDE_PATHS_))),)
  # No librealsense2 but there is a librealsense.
  # Check its version by checking for RS_API_MAJOR_VERSION in the header.
  REALSENSE_VERSION=$(shell awk \
    '/^\#define\s+RS2_API_MAJOR_VERSION\s+\S+/{ print $$3; }' \
    $(firstword $(wildcard $(addsuffix /rs.h,$(REALSENSE_INCLUDE_PATHS_)))))
  ifeq ($(REALSENSE_VERSION),2)
    LIBS_realsense2 += realsense2
    PLUGINS_build = $(PLUGINS_all)
  else
    WARN_TARGETS += warning_librealsense_version
  endif
else
  WARN_TARGETS += warning_librealsense
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_librealsense warning_librealsense_version
warning_librealsense:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense Plugin$(TNORMAL) (librealsense2 not found)"
warning_librealsense_version:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense Plugin$(TNORMAL) (incompatible librealsense version $(REALSENSE_VERSION))"
endif

include $(BUILDSYSDIR)/base.mk
