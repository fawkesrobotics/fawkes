#*****************************************************************************
#         Makefile Build System for Fawkes: realsense Plugin
#                            -------------------
#   Created on Mon Jun 13 17:09:44 2016
 
#   Copyright (C) 2016 by Johannes Rothe
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
                      fawkesblackboard fawkesinterface realsense usb-1.0 \
                      fawkespcl_utils

OBJS_realsense = realsense_plugin.o realsense_thread.o 

ifneq ($(wildcard $(SYSROOT)/usr/local/lib/librealsense.so),)
	PLUGINS_all = $(PLUGINDIR)/realsense.$(SOEXT)
else
	WARN_TARGETS += warning_librealsense
endif

OBJS_all = $(OBJS_realsense)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_nolibrealsense
warning_librealsense:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense Plugin$(TNORMAL) (librealsense not found)"
endif

include $(BUILDSYSDIR)/base.mk