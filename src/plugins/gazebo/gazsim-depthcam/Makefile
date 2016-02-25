#*****************************************************************************
#               Makefile Build System for Fawkes: Gazsim-depthcam Plugin
#
#   Created on Sat Sep 21 17:32:31 2013
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
#                         Frederik Zwilling
#
##*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BUILDSYSDIR)/protobuf.mk
# include $(BASEDIR)/src/plugins/gazebo/protobuf.mk
include $(BUILDSYSDIR)/pcl.mk

LIBS_gazsim_depthcam = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       fawkesgazeboaspect
OBJS_gazsim_depthcam = gazsim_depthcam_plugin.o gazsim_depthcam_thread.o

OBJS_all    = $(OBJS_gazsim_depthcam)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(HAVE_TF)$(HAVE_PCL),1111)
  CFLAGS  += $(CFLAGS_TF)
  LDFLAGS += $(LDFLAGS_TF)
  CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF) $(CFLAGS_PCL)
  LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) $(LDFLAGS_PCL) -lm $(call boost-libs-ldflags,system) -lboost_system

  PLUGINS_all = $(PLUGINDIR)/gazsim-depthcam.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_gazebo warning_protobuf
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-depthcam Plugin$(TNORMAL) " \
                         "(tf not available)"
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-depthcam Plugin$(TNORMAL) " \
		"(Gazebo Simulator not found)"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-depthcam Plugin$(TNORMAL) " \
		"(protobuf[-devel] not installed)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting gazsim-depthcam Plugin (pcl[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk
