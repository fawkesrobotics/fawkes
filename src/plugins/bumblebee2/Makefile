#*****************************************************************************
#         Makefile Build System for Fawkes: Bumblebee2 Plugin
#                            -------------------
#   Created on Wed Jul 17 13:11:30 2013
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/fvconf.mk
include $(BUILDCONFDIR)/tf/tf.mk

# Enable to get time tracking data
# CFLAGS += -DUSE_TIMETRACKER

PRESUBDIRS = interfaces
SUBDIRS = calib

LIBS_bumblebee2 = fawkescore fawkesutils fawkesaspects fvutils fvcams \
		  fawkestf fawkesinterface fawkesblackboard \
		  SwitchInterface OpenCVStereoParamsInterface \
		  opencv_core opencv_calib3d
OBJS_bumblebee2 = bumblebee2_plugin.o bumblebee2_thread.o

OBJS_all    = $(OBJS_bumblebee2)

ifeq ($(HAVE_PCL)$(HAVE_TF)$(HAVE_TRICLOPS)$(HAVE_OPENCV),1111)
  CFLAGS  += $(CFLAGS_TF) $(CFLAGS_PCL) $(CFLAGS_TRICLOPS) $(CFLAGS_OPENCV)
  LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) $(LDFLAGS_TRICLOPS)
  PLUGINS_all = $(PLUGINDIR)/bumblebee2.$(SOEXT)
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_TRICLOPS),1)
    WARN_TARGETS += warning_triclops
  endif
  ifneq ($(HAVE_OPENCV),1)
    WARN_TARGETS += warning_opencv
  endif
endif

# Required for weirdo PCL (on Fedora with ROS headers)
#CFLAGS  += $(call boost-lib-cflags,system)
#LDFLAGS += $(call boost-lib-ldflags,system)



ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_triclops warning_opencv
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting bumblebee2 plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting bumblebee2 plugin$(TNORMAL) (PCL not available)"
warning_triclops:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting bumblebee2 plugin$(TNORMAL) (Triclops not found)"
warning_opencv:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting bumblebee2 plugin$(TNORMAL) (OpenCV not found)"
endif

include $(BUILDSYSDIR)/base.mk

