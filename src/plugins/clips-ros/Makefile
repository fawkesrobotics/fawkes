#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS ROS Integration
#                            -------------------
#   Created on Tue Oct 22 18:06:01 2013
#   Copyright (C) 2006-2013 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/clips.mk

CFLAGS += $(CFLAGS_CPP11)

LIBS_clips_ros = fawkescore fawkesutils fawkesaspects fawkesclipsaspect \
		 fawkesrosaspect
OBJS_clips_ros = clips_ros_plugin.o clips_ros_thread.o

OBJS_all = $(OBJS_clips_ros)

ifeq ($(HAVE_ROS)$(HAVE_CLIPS)$(call ros-have-pkg,xmlrpcpp),111)
  CFLAGS  += $(CFLAGS_ROS)  $(CFLAGS_CLIPS) $(call ros-pkg-cflags,xmlrpcpp)
  LDFLAGS += $(LDFLAGS_ROS) $(LDFLAGS_CLIPS) $(call ros-pkg-lflags,xmlrpcpp)

  PLUGINS_all = $(PLUGINDIR)/clips-ros.$(SOEXT)
else
  ifneq ($(HAVE_ROS),1)
    WARN_TARGETS += warning_ros
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS += warning_clips
  endif
  ifneq ($(call ros-have-pkg,xmlrpcpp),1)
    WARN_TARGETS += warning_xmlrpcpp
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_ros warning_clips warning_xmlrpcpp
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) (ROS not found)"

warning_xmlrpcpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) (xmlrpcpp ROS package not found)"

warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) (CLIPS not found)"
endif

include $(BUILDSYSDIR)/base.mk
