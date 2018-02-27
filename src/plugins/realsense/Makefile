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

LIBS_realsense = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface \
                      fawkespcl_utils SwitchInterface

OBJS_realsense = realsense_plugin.o realsense_thread.o 

__REALSENSE1_INCLUDE_PATHS = /usr/include/librealsense1 /usr/local/include/librealsense1
__REALSENSE_INCLUDE_PATHS = /usr/include/librealsense /usr/local/include/librealsense

ifneq ($(wildcard $(addsuffix /rs.h,$(__REALSENSE1_INCLUDE_PATHS))),)
	# The old librealsense has been moved to librealsense1, librealsense is now
	# librealsense2.  We need the old version because the newer version does not
	# support our camera.
  CFLAGS += -DHAVE_REALSENSE1
  LIBS_realsense += realsense1
  PLUGINS_all = $(PLUGINDIR)/realsense.$(SOEXT)
else ifneq ($(wildcard $(addsuffix /rs.h,$(__REALSENSE_INCLUDE_PATHS))),)
	# No librealsense1 but there is a librealsense.
	# Check its version by checking for RS_API_MAJOR_VERSION in the header.
  REALSENSE_VERSION=$(shell awk \
    '/^\#define\s+RS_API_MAJOR_VERSION\s+\S+/{ print $$3; }' \
    $(firstword $(wildcard $(addsuffix /rs.h,$(__REALSENSE_INCLUDE_PATHS)))))
  ifeq ($(REALSENSE_VERSION),1)
    LIBS_realsense += realsense
    PLUGINS_all = $(PLUGINDIR)/realsense.$(SOEXT)
  else
    WARN_TARGETS += warning_librealsense_version
  endif
else
  WARN_TARGETS += warning_librealsense
endif

OBJS_all = $(OBJS_realsense)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_librealsense warning_librealsense_version
warning_librealsense:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense Plugin$(TNORMAL) (librealsense not found)"
warning_librealsense_version:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense Plugin$(TNORMAL) (incompatible librealsense version $(REALSENSE_VERSION))"
endif

include $(BUILDSYSDIR)/base.mk
