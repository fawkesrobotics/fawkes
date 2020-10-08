#*****************************************************************************
#         Makefile Build System for Fawkes: realsense Plugin
#                            -------------------
#   Created on Mon Jun 13 17:09:44 2016
#   Copyright (C) 2016 by Johannes Rothe
#                 2018 by Till Hofmann
#                 2019 by Christoph Gollok
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

CFLAGS  +=  $(CFLAGS_CPP11)
CFLAGS  +=  $(CFLAGS_TF) $(CFLAGS_PCL) $(CFFLAGS_conveyor_pose)
LDFLAGS +=  $(LDFLAGS_TF) $(LDFLAGS_PCL)

LIBS_realsense2 = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface \
                      fawkespcl_utils SwitchInterface CameraControlInterface

OBJS_realsense2 = realsense2_plugin.o realsense2_thread.o 

OBJS_all    = $(OBJS_realsense2)
PLUGINS_all = $(PLUGINDIR)/realsense2.$(SOEXT)

REALSENSE_MIN_VERSION = 2.32.0

ifeq ($(shell $(PKGCONFIG) --exists realsense2; echo $$?),0)
    ifeq ($(shell $(PKGCONFIG) --atleast-version $(REALSENSE_MIN_VERSION) realsense2; echo $$?),0)
	CFLAGS  += $(shell $(PKGCONFIG) --cflags realsense2)
	LDFLAGS += $(shell $(PKGCONFIG) --libs realsense2)
        LIBS_realsense2 += realsense2
        PLUGINS_build = $(PLUGINS_all)
    else
        WARN_TARGETS += warning_librealsense_version
	REALSENSE_VERSION = $(shell $(PKGCONFIG) --modversion realsense2)
    endif
else
    WARN_TARGETS += warning_librealsense
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_librealsense warning_librealsense_version
warning_librealsense:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense2 Plugin$(TNORMAL) (librealsense2 not found)"
warning_librealsense_version:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Realsense2 Plugin$(TNORMAL) (incompatible librealsense version $(REALSENSE_VERSION); required >= $(REALSENSE_MIN_VERSION))"
endif

include $(BUILDSYSDIR)/base.mk
