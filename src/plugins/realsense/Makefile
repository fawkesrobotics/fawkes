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

PLUGINS_all = $(PLUGINDIR)/realsense.$(SOEXT)

OBJS_all = $(OBJS_realsense)

include $(BUILDSYSDIR)/base.mk